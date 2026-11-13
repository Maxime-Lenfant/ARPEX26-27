#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <driver/twai.h>
#include <math.h>

// Config materielle
// ==========================================
Adafruit_ADS1115 ads;

// --- Broches I2C ---
const int PIN_SDA = 21; // D21 dans le PDF
const int PIN_SCL = 22; // D22 dans le PDF

// --- Broches UART (Vers le Pico) ---
const int PICO_RX = 16; // RX2 (GPIO 16) dans le PDF
const int PICO_TX = 17; // TX2 (GPIO 17) dans le PDF

// --- Broches CAN (Sélecteur de vitesse) ---
const gpio_num_t CAN_TX = GPIO_NUM_4; // D4 dans le PDF
const gpio_num_t CAN_RX = GPIO_NUM_5; // D5 dans le PDF

// Variables Globales
// ==========================================
const float PEDALE_V_MIN_C1 = 1.5;
const float PEDALE_V_MAX_C1 = 3.3;
const float PEDALE_V_MIN_C2 = 0.8;
const float PEDALE_V_MAX_C2 = 3.7;
const float PEDALE_MAX_OUT = 4.5;
const int RPM_MOTEUR_MAX = 8000;

enum ModeConduite { MODE_INIT = 0, MODE_AUTO = 1, MODE_SIMU = 2 };
enum EtatSysteme { ERR_OK = 0, ERR_CAP = 1, ERR_ALL = 2 };

ModeConduite mode_conduite = MODE_INIT;
EtatSysteme etat_systeme = ERR_OK;

struct ParamRapport {
    float rpm_max;
    float k;
    float v0;
};

const ParamRapport Parametre_Rapport[7] = {
    {0, 0, 0},
    {1739, 4.113, 1.126},
    {2956, 3.673, 1.302},
    {4173, 3.673, 1.445},
    {5389, 2.706, 1.588},
    {6597, 2.337, 1.760},
    {7960, 2.057, 1.932}
};

uint8_t rapport_actuel = 1;
uint8_t dernier_rapport_connu = 1;
float rpm_reel = 0.0;
float voltage_calcul_interne = 0.0;

// État des boutons provenant du Pico
uint8_t etat_clignotants = 0;
uint8_t etat_radio = 0;

unsigned long dernierTemps = 0;
const unsigned long PERIODE_BOUCLE = 20;

// Initialisation des bus
void init_bus_can() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        Serial.println("Bus CAN initialise sur TX=4 et RX=5.");
    } else {
        Serial.println("ERREUR CRITIQUE: Echec initialisation Bus CAN.");
    }
}

void setup() {
    // Debug Série (TX0/RX0)
    Serial.begin(115200);
    delay(1000); // Temps de respiration indispensable au boot
    Serial.println("\n=== DEMARRAGE ESP32 CONTROLEUR CENTRAL ===");

    // UART vers Pico (TX2/RX2)
    Serial1.begin(115200, SERIAL_8N1, PICO_RX, PICO_TX);
    Serial1.setTimeout(0); 
    Serial.println("UART1 initialise sur RX=16 et TX=17");

    // Initialisation I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    if (!ads.begin()) {
        Serial.println("Attention : ADS1115 introuvable");
    } else {
        Serial.println("ADS1115 trouve.");
    }

    // Initialisation du CAN
    init_bus_can();
    
    Serial.println("Setup OK");
}

float lire_pedale_brute() {
    float v1 = ads.computeVolts(ads.readADC_SingleEnded(0));
    float v2 = ads.computeVolts(ads.readADC_SingleEnded(1));
    Serial.println("Tension récup V1 " + String(v1));
    if (v1 < (PEDALE_V_MIN_C1 - 0.2) || v2 < (PEDALE_V_MIN_C2 - 0.2)) {
        etat_systeme = ERR_CAP;
        return 0.0;
    }

    float ratio_v1 = (v1 - PEDALE_V_MIN_C1) / (PEDALE_V_MAX_C1 - PEDALE_V_MIN_C1);
    float ratio_v2 = (v2 - PEDALE_V_MIN_C2) / (PEDALE_V_MAX_C2 - PEDALE_V_MIN_C2);
    if (abs(constrain(ratio_v1, 0.0, 1.0) - constrain(ratio_v2, 0.0, 1.0)) > 0.3) {
        etat_systeme = ERR_CAP;
        return v1;
    }
    etat_systeme = ERR_OK;
    return v1;
}

float calcul_rpm_manuel(float pedale_v, uint8_t rapport) {
    if (rapport < 1 || rapport > 6 || pedale_v < 0.1) return 0;
    ParamRapport p = Parametre_Rapport[rapport];
    return p.rpm_max / (1.0 + exp(-p.k * (pedale_v - p.v0)));
}

float calcul_voltage_inverse(float target_rpm, uint8_t rapport) {
    if (rapport < 1 || rapport > 6 || target_rpm <= 100) return 0;
    ParamRapport p = Parametre_Rapport[rapport];
    if (target_rpm >= p.rpm_max * 0.99) return PEDALE_MAX_OUT;

    float terme = (p.rpm_max / target_rpm) - 1.0;
    if (terme <= 0) return PEDALE_MAX_OUT;
    return constrain(p.v0 - (1.0 / p.k) * log(terme), 0.0, PEDALE_MAX_OUT);
}

void lire_bus_can() {
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) {
        if (message.identifier == 0x100 && message.data_length_code >= 1) {
            mode_conduite = (ModeConduite)message.data[0];
            if (message.data_length_code >= 2) {
                rapport_actuel = constrain(message.data[1], 1, 6);
            }
        }
    }
}

void gerer_communications_pico() {
    Serial.println(String(Serial1.available())); //LA
    while (Serial1.available() >= 2) {
        uint8_t buffer[2];
        Serial1.readBytes(buffer, 2);
        etat_clignotants = buffer[0];
        etat_radio = buffer[1];
    }
    
    uint16_t rpm_int = (uint16_t)rpm_reel;
    uint8_t trame_out[4] = {
        (uint8_t)mode_conduite,
        rapport_actuel,
        (uint8_t)((rpm_int >> 8) & 0xFF),
        (uint8_t)(rpm_int & 0xFF)
    };
    Serial1.write(trame_out, 4);
}

void loop() {
    unsigned long tempsActuel = millis();
    
    if (tempsActuel - dernierTemps >= PERIODE_BOUCLE) {
        dernierTemps = tempsActuel;

        //lire_bus_can();
        float tension_brute = lire_pedale_brute();
        float pedale_physique = constrain((tension_brute - 1.64) / (3.74 - 1.64) * 4.5, 0.0, 4.5);
        if (pedale_physique < 0.1) pedale_physique = 0;

        float target_rpm = 0;

        if (mode_conduite == MODE_INIT) {
            target_rpm = 0;
            voltage_calcul_interne = pedale_physique;   
        } 
        else if (mode_conduite == MODE_AUTO) {
            if (pedale_physique > 0.1) target_rpm = (pedale_physique / PEDALE_MAX_OUT) * RPM_MOTEUR_MAX;
            target_rpm = min(target_rpm, (float)RPM_MOTEUR_MAX);
            voltage_calcul_interne = pedale_physique;
        } 
        else if (mode_conduite == MODE_SIMU) {
            if (rapport_actuel != dernier_rapport_connu) {
                voltage_calcul_interne = calcul_voltage_inverse(rpm_reel, rapport_actuel);
                dernier_rapport_connu = rapport_actuel;
            }
            float ecart = pedale_physique - voltage_calcul_interne;
            if (abs(ecart) > 0.05) voltage_calcul_interne += ecart * 0.2; 
            else voltage_calcul_interne = pedale_physique;
                
            target_rpm = calcul_rpm_manuel(voltage_calcul_interne, rapport_actuel);
        }
        rpm_reel = target_rpm;

        gerer_communications_pico();
        
    }
    //delay pour le repos
    yield();
    delay(2);
}