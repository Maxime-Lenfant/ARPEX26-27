#include <TFT_eSPI.h>
#include <SPI.h>

/* ================== TFT ================== */
TFT_eSPI tft = TFT_eSPI();

#define LARGEUR_ECRAN 480
#define HAUTEUR_ECRAN 320

#define COULEUR_ORANGE 0xFC60
#define COULEUR_ROUGE  0xF800
#define COULEUR_BLEU   0x001F
#define COULEUR_FOND   COULEUR_ORANGE
#define COULEUR_TEXTE  TFT_BLACK

/* ================== PINS ================== */
const int PIN_MUTE           = 2;
const int PIN_PALETTE_GAUCHE = 3;
const int PIN_APPEL_PHARES   = 4;
const int PIN_RADIO          = 5;
const int PIN_PHARES         = 6;
const int PIN_PALETTE_DROITE = 11;
const int PIN_CLIGNO_D       = 12;
const int PIN_CLIGNO_G       = 13;

/* ================== MODES ================== */
enum ModeGlobal { MODE_INIT, MODE_RACE };
enum ModeRace   { RACE_ELEC, RACE_SIMU };

ModeGlobal modeGlobal = MODE_INIT;
ModeRace   modeRace   = RACE_ELEC;

/* ================== VARIABLES ================== */
int rapport = 1;          
int rapport_ui_prec  = -1;
int rapport_usb_prec = -1;
int pedalePourcentage = 0;

unsigned long ui_t0 = 0;
const unsigned long UI_PERIOD_MS = 100;

/* ================== VARIABLES D'ÉTAT ================== */
ModeGlobal modeGlobal_prec = (ModeGlobal)-1;
ModeRace   modeRace_prec   = (ModeRace)-1;

/* ================== OUTILS UI ================== */
String etat_on_off(int v) { return (v == LOW) ? "On " : "Off"; }

void drawStringCentered(const String& txt, int y, int font, int x_offset = 0) {
    int16_t x = (tft.width() - tft.textWidth(txt, font)) / 2 + x_offset;
    tft.drawString(txt, x, y, font);
}

/* ================== COMMUNICATION BINAIRE (ESP32) ================== */
void envoyerTrameBinaire(bool eventChangement = false) {
    uint8_t trame[2] = {0x00, 0x00};

    // --- OCTET 1 : États des boutons (1 = Appuyé / Actif) ---
    if (digitalRead(PIN_CLIGNO_G) == LOW)       trame[0] |= (1 << 0);
    if (digitalRead(PIN_CLIGNO_D) == LOW)       trame[0] |= (1 << 1);
    if (digitalRead(PIN_PHARES) == LOW)         trame[0] |= (1 << 2);
    if (digitalRead(PIN_APPEL_PHARES) == LOW)   trame[0] |= (1 << 3);
    if (digitalRead(PIN_RADIO) == LOW)          trame[0] |= (1 << 4);
    if (digitalRead(PIN_MUTE) == LOW)           trame[0] |= (1 << 5);
    if (digitalRead(PIN_PALETTE_GAUCHE) == LOW) trame[0] |= (1 << 6);
    if (digitalRead(PIN_PALETTE_DROITE) == LOW) trame[0] |= (1 << 7);

    // --- OCTET 2 : Mode et Rapport ---
    // Bits 0..3 : Rapport (1 à 6)
    trame[1] |= (rapport & 0x0F);

    // Bits 4..5 : Mode de conduite (0 = INIT, 1 = ELEC, 2 = SIMU)
    uint8_t codeMode = 0;
    if (modeGlobal == MODE_INIT) {
        codeMode = 0;
    } else if (modeRace == RACE_ELEC) {
        codeMode = 1;
    } else {
        codeMode = 2;
    }
    trame[1] |= ((codeMode & 0x03) << 4);

    // Bit 7 : Flag d'évènement / changement de rapport
    if (eventChangement) {
        trame[1] |= 0x80;
    }

    // Envoi de la trame binaire sur UART1 (Pin GP0 TX vers ESP32 RX)
    Serial1.write(trame, 2);
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);   // Debug PC via USB
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) {
    delay(10);
  }
  Serial.println("SERIE = OKK");
  Serial1.begin(115200);  // Liaison UART vers l'ESP32-S3 (GP0/GP1)

  tft.init();
  tft.setRotation(1);
  tft.setTextFont(2);       
  tft.setTextColor(TFT_BLACK, COULEUR_FOND);
  tft.fillScreen(COULEUR_FOND);

  pinMode(PIN_PALETTE_DROITE, INPUT_PULLUP);
  pinMode(PIN_PALETTE_GAUCHE, INPUT_PULLUP);
  pinMode(PIN_RADIO, INPUT_PULLUP);
  pinMode(PIN_APPEL_PHARES, INPUT_PULLUP);
  pinMode(PIN_CLIGNO_G, INPUT_PULLUP);
  pinMode(PIN_CLIGNO_D, INPUT_PULLUP);
  pinMode(PIN_MUTE, INPUT_PULLUP);
  pinMode(PIN_PHARES, INPUT_PULLUP);

  int16_t text_width = tft.textWidth("ARPEX", 4);
  int16_t x = (LARGEUR_ECRAN - text_width) / 2;
  int16_t y = HAUTEUR_ECRAN / 2 - 12; 
  tft.drawString("ARPEX", x, y, 4);
  delay(2000);
  tft.fillScreen(COULEUR_FOND);
}

/* ================== GESTION DES MODES ================== */
void gerer_changement_mode_global() {
  static unsigned long t0 = 0;
  static int prev = HIGH;
  int d = digitalRead(PIN_PALETTE_DROITE);
  unsigned long now = millis();
  if (d == LOW && prev == HIGH) t0 = now;
  if (d == HIGH && prev == LOW && now - t0 > 2000) {
    modeGlobal = (modeGlobal == MODE_INIT) ? MODE_RACE : MODE_INIT;
    tft.fillScreen(COULEUR_FOND);
  }
  prev = d;
}

void gerer_mode_race() {
  static unsigned long t0 = 0;
  static int prev = HIGH;
  int g = digitalRead(PIN_PALETTE_GAUCHE);
  unsigned long now = millis();
  if (g == LOW && prev == HIGH) t0 = now;
  if (g == HIGH && prev == LOW && now - t0 > 2000) {
    if (modeRace == RACE_ELEC)
      modeRace = RACE_SIMU;
    else 
      modeRace = RACE_ELEC;
    tft.fillScreen(COULEUR_FOND);
  }
  prev = g;
}

/* ================== RAPPORT ================== */
void gerer_rapport() {
    static int pg_prev = HIGH;
    static int pd_prev = HIGH;
    static unsigned long t_last = 0;
    const unsigned long DEBOUNCE_MS = 200;
    int pg = digitalRead(PIN_PALETTE_GAUCHE);
    int pd = digitalRead(PIN_PALETTE_DROITE);
    unsigned long now = millis();
    
    if (now - t_last > DEBOUNCE_MS) {
        if (pd == LOW && pd_prev == HIGH && rapport < 6) {
            rapport++;
            t_last = now;
            rapport_ui_prec = -1;   
            envoyerTrameBinaire(true); 
        }
        if (pg == LOW && pg_prev == HIGH && rapport > 1) {
            rapport--;
            t_last = now;
            rapport_ui_prec = -1;   
            envoyerTrameBinaire(true);  
        }
    }
    pg_prev = pg;
    pd_prev = pd;
}

/* ================== AFFICHAGE INIT ================== */
void draw_init() {
  tft.fillScreen(COULEUR_FOND);
  tft.drawString("INITIALISATION", 20, 20, 4);
  drawStringCentered("Mute : ",          60, 2);
  drawStringCentered("CLIGNO G : ",        90, 2);
  drawStringCentered("CLIGNO D : ",       120, 2);
  drawStringCentered("Palette G : ",      150, 2);
  drawStringCentered("Palette D : ",      180, 2);
  drawStringCentered("Appel phares : ",   210, 2);
  drawStringCentered("Radio : ",          240, 2);
  drawStringCentered("PHARES : ",         270, 2);
}

void drawEtatInitSafe(String txt, int y, int x_offset = 80) {
  int16_t x = (tft.width() - 60) / 2 + x_offset;
  tft.fillRect(x, y, 60, 20, COULEUR_FOND);
  tft.setTextColor(TFT_BLACK, COULEUR_FOND);
  tft.drawString(txt, x, y);
}

void update_inputs_init(bool force = false) {
  if (modeGlobal != MODE_INIT) return; 
  static int mute_prec      = -1;
  static int cligno_g_prec  = -1;
  static int cligno_d_prec  = -1;
  static int palette_g_prec = -1;
  static int palette_d_prec = -1;
  static int appel_prec     = -1;
  static int radio_prec     = -1;
  static int phares_prec    = -1;

  if (force) {
    mute_prec      = -1;
    cligno_g_prec  = -1;
    cligno_d_prec  = -1;
    palette_g_prec = -1;
    palette_d_prec = -1;
    appel_prec     = -1;
    radio_prec     = -1;
    phares_prec    = -1;
  }
  int mute      = digitalRead(PIN_MUTE);
  int cligno_g  = digitalRead(PIN_CLIGNO_G);
  int cligno_d  = digitalRead(PIN_CLIGNO_D);
  int palette_g = digitalRead(PIN_PALETTE_GAUCHE);
  int palette_d = digitalRead(PIN_PALETTE_DROITE);
  int appel     = digitalRead(PIN_APPEL_PHARES);
  int radio     = digitalRead(PIN_RADIO);
  int phares    = digitalRead(PIN_PHARES);

  tft.startWrite();
  if (mute != mute_prec) {
    drawEtatInitSafe(etat_on_off(mute), 60);
    mute_prec = mute;
  }
  if (cligno_g != cligno_g_prec) {
    drawEtatInitSafe(etat_on_off(cligno_g), 90);
    cligno_g_prec = cligno_g;
  }
  if (cligno_d != cligno_d_prec) {
    drawEtatInitSafe(etat_on_off(cligno_d), 120);
    cligno_d_prec = cligno_d;
  }
  if (palette_g != palette_g_prec) {
    drawEtatInitSafe(etat_on_off(palette_g), 150);
    palette_g_prec = palette_g;
  }
  if (palette_d != palette_d_prec) {
    drawEtatInitSafe(etat_on_off(palette_d), 180);
    palette_d_prec = palette_d;
  }
  if (appel != appel_prec) {
    drawEtatInitSafe(etat_on_off(appel), 210);
    appel_prec = appel;
  }
  if (radio != radio_prec) {
    drawEtatInitSafe(etat_on_off(radio), 240);
    radio_prec = radio;
  }
  if (phares != phares_prec) {
    drawEtatInitSafe(etat_on_off(phares), 270);
    phares_prec = phares;
  }
  tft.endWrite();
}

enum EtatSysteme {
  ETAT_OK,
  ETAT_CAP_MIN,
  ETAT_CRITIQUE
};
EtatSysteme etatCourant = ETAT_OK;

void refreshPedale() {
    if (modeGlobal != MODE_RACE) return;
    static int dernierPourcentage = -1;
    if (pedalePourcentage != dernierPourcentage) {
        tft.startWrite();
        int x = 370;
        int y = 20;
        int h = 24;   
        int font = 2;
        int w = tft.textWidth("100", font);
        tft.setTextFont(2);
        tft.fillRect(x, y, w, h, COULEUR_FOND);
        tft.setTextColor(TFT_BLACK);
        tft.setCursor(x, y);
        tft.print(pedalePourcentage);
        tft.setCursor(x + w + 2, y); 
        tft.print("%");
        tft.endWrite();
        dernierPourcentage = pedalePourcentage;
    }
}

#define MSG_X  10
#define MSG_Y  (HAUTEUR_ECRAN - 50)
#define MSG_W  200
#define MSG_H  40

unsigned long message_debut = 0;
const unsigned long MESSAGE_DURATION = 3000;
bool message_visible = false;

void afficherMessage() {
    static bool visible = true;
    static unsigned long precedent = 0;
    unsigned long now = millis();

    if (etatCourant == ETAT_OK) {
        tft.fillRect(MSG_X, MSG_Y, MSG_W, MSG_H, COULEUR_FOND);
        return;
    }

    if (message_visible && now - message_debut > MESSAGE_DURATION) {
        tft.fillRect(MSG_X, MSG_Y, MSG_W, MSG_H, COULEUR_FOND);
        message_visible = false;
        etatCourant = ETAT_OK;
        return;
    }

    if (now - precedent >= 500) {
        precedent = now;
        visible = !visible;
        tft.fillRect(MSG_X, MSG_Y, MSG_W, MSG_H, COULEUR_FOND);

        if (visible) {
            if (etatCourant == ETAT_CAP_MIN)
                tft.setTextColor(TFT_GREEN, COULEUR_FOND);
            else if (etatCourant == ETAT_CRITIQUE)
                tft.setTextColor(TFT_RED, COULEUR_FOND);
            tft.drawString(
                (etatCourant == ETAT_CAP_MIN) ? "MODE DEGRADE" : "ARRET IMMEDIAT",
                MSG_X + 10, MSG_Y + 10, 4
            );
        }
    }
}

void liredonnee() {
    // Lecture des données télémétriques provenant de l'ESP32
    if (Serial1.available() >= 4) {
        uint8_t buf[4];
        Serial1.readBytes(buf, 4);
        // Ex: buf[0]=mode, buf[1]=rapport, buf[2]=RPM High, buf[3]=RPM Low
    }
}

void draw_race_static() {
  tft.fillScreen(COULEUR_FOND);
  tft.drawString("RACE", 10, 20, 4);
  tft.drawString("GAUCHE", 10, 120,4);
  tft.drawString("DROIT", 380, 120,4);
  tft.drawString("PHARES", 40, 200);
  tft.drawString("ADP", 40, 240);
  tft.drawString("RADIO", 400, 200);
  tft.drawString("MUTE", 400, 240);
  tft.fillRect(190, 20, 200, 24, COULEUR_FOND);
}

void drawModeRace() {
    tft.fillRect(190, 20, 200, 24, COULEUR_FOND);
    if (modeRace == RACE_ELEC)
        tft.drawString("MODE ELECTRIQUE", 190, 20);
    else
        tft.drawString("MODE SIMULATION", 190, 20);
}

bool force_redraw_race = false;

void update_race_dynamic() {
    if (modeGlobal != MODE_RACE) return;
    static int cligno_g_prec = -1;
    static int cligno_d_prec = -1;
    static int phares_prec   = -1;
    static int appel_prec    = -1;
    static int radio_prec    = -1;
    static int mute_prec     = -1;

    if (force_redraw_race) {
      cligno_g_prec = -1;
      cligno_d_prec = -1;
      phares_prec   = -1;
      appel_prec    = -1;
      radio_prec    = -1;
      mute_prec     = -1;
      rapport_ui_prec = -1;
      force_redraw_race = false;
    }

    int cligno_g = digitalRead(PIN_CLIGNO_G);
    int cligno_d = digitalRead(PIN_CLIGNO_D);
    int phares   = digitalRead(PIN_PHARES);
    int appel    = digitalRead(PIN_APPEL_PHARES);
    int radio    = digitalRead(PIN_RADIO);
    int mute     = digitalRead(PIN_MUTE);

    tft.startWrite();
    if (cligno_g != cligno_g_prec) {
        tft.fillRect(10, 120, 120, 30, COULEUR_FOND);
        tft.setTextFont(4);
        tft.setTextColor(cligno_g == LOW ? COULEUR_ROUGE : TFT_BLACK, COULEUR_FOND);
        tft.drawString("GAUCHE", 10, 120);
        cligno_g_prec = cligno_g;
    }
    if (cligno_d != cligno_d_prec) {
        tft.fillRect(380, 120, 120, 30, COULEUR_FOND);
        tft.setTextFont(4);
        tft.setTextColor(cligno_d == LOW ? COULEUR_ROUGE : TFT_BLACK, COULEUR_FOND);
        tft.drawString("DROIT", 400, 120);
        cligno_d_prec = cligno_d;
    }
    if (phares != phares_prec) {
        tft.fillRect(40, 200, 120, 24, COULEUR_FOND);
        tft.setTextFont(2);
        tft.setTextColor(phares == LOW ? COULEUR_BLEU : TFT_BLACK, COULEUR_FOND);
        tft.drawString("PHARES", 40, 200);
        phares_prec = phares;
    }
    if (appel != appel_prec) {
        tft.fillRect(40, 240, 120, 24, COULEUR_FOND);
        tft.setTextFont(2);
        tft.setTextColor(appel == LOW ? COULEUR_BLEU : TFT_BLACK, COULEUR_FOND);
        tft.drawString("ADP", 40, 240);
        appel_prec = appel;
    }
    if (radio != radio_prec) {
        tft.fillRect(400, 200, 120, 24, COULEUR_FOND);
        tft.setTextFont(2);
        tft.setTextColor(radio == LOW ? COULEUR_BLEU : TFT_BLACK, COULEUR_FOND);
        tft.drawString("RADIO", 400, 200);
        radio_prec = radio;
    }
    if (mute != mute_prec) {
        tft.fillRect(400, 240, 120, 24, COULEUR_FOND);
        tft.setTextFont(2);
        tft.setTextColor(mute == LOW ? COULEUR_BLEU : TFT_BLACK, COULEUR_FOND);
        tft.drawString("MUTE", 400, 240);
        mute_prec = mute;
    }
    if (modeRace == RACE_SIMU && rapport != rapport_ui_prec) {
        tft.fillRect(0, 140, LARGEUR_ECRAN, 60, COULEUR_FOND);
        tft.setTextFont(8);
        tft.setTextColor(TFT_BLACK, COULEUR_FOND);
        drawStringCentered(String(rapport), 150, 8);
        rapport_ui_prec = rapport;
    }
    tft.endWrite();
}

/* ================== LOOP ================== */
void loop() {
    gerer_changement_mode_global();
    liredonnee();
    refreshPedale();
    afficherMessage();

    if (modeGlobal == MODE_RACE)
        gerer_mode_race();

    /* ===== Changement de MODE GLOBAL ===== */
    if (modeGlobal != modeGlobal_prec) {
        modeGlobal_prec = modeGlobal;
        if (modeGlobal == MODE_INIT) {
            draw_init();
            update_inputs_init(true); 
        } else {
            draw_race_static();
            drawModeRace();
            force_redraw_race = true;
            rapport = 1;
            rapport_ui_prec = rapport_usb_prec = -1;
        }
        envoyerTrameBinaire(true);
    }

    /* ===== Changement de SOUS-MODE RACE ===== */
    if (modeGlobal == MODE_RACE && modeRace != modeRace_prec) {
        modeRace_prec = modeRace;
        rapport = 1;
        rapport_ui_prec = -1;
        rapport_usb_prec = -1;
        draw_race_static();
        drawModeRace();
        force_redraw_race = true;
        envoyerTrameBinaire(true);
    }

    /* ===== Rafraîchissement périodique UI et Envoi Télémétrie ===== */
    if (millis() - ui_t0 > UI_PERIOD_MS) {
        ui_t0 = millis();

        refreshPedale();
        if (modeGlobal == MODE_INIT) {
            update_inputs_init();
        } else {
            update_race_dynamic();
            gerer_rapport();
        }

        // Transmet en continu l'état des boutons et des modes à l'ESP32
        envoyerTrameBinaire(false);
    }
    afficherMessage();
}