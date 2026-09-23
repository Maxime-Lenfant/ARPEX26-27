# =============================================================================
# PROJET VOLANT F1 - CODE FINAL
# =============================================================================

import RPi.GPIO as GPIO
import time
import math
import threading
import serial

# --- Imports pour l'ADS1115 (Validés par ton test) ---
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# ==========================
# 1. CONFIGURATION DU MATÉRIEL
# ==========================

# --- Moteur ---
PIN_MOTOR_PWM = 12

# --- Levier (Virtuel pour l'instant) ---
PIN_P, PIN_R, PIN_N, PIN_D = 22, 23, 24, 25

PIN_URGENCE= 17

# --- Communication Pico ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- Configuration ADC ---

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
# Mais on garde la création des canaux comme dans ton test :
chan_pedale_1 = AnalogIn(ads, 0) # Pédale Principale
chan_pedale_2 = AnalogIn(ads, 1) # Pédale Sécurité

# --- Calibration Pédale  ---
PEDALE_V_MIN_REEL = 0.62  # Tension pied levé
PEDALE_V_MAX_REEL = 3.15  # Tension pied à fond
PEDALE_ZONE_MORTE = 0.1
PEDALE_MAX_OUT = 4.5     # Tension virtuelle pour les maths
PEDALE_V_MIN_C1=1.5
PEDALE_V_MAX_C1=3.3
PEDALE_V_MIN_C2=0.5
PEDALE_V_MAX_C2=3.7


# --- Sécurité ---
MAX_ECART = 0.3
ERR_OK = 'OK'
ERR_ALL = 'CRITIQUE'
ERR_CAP = 'CAP_MIN'

# --- Paramètres Moteur ---
RPM_MOTEUR_MAX = 8000
MODE_INIT = 0
MODE_AUTO = 1
MODE_SIMU = 2

Parametre_Rapport = {
    1: {'rpm_max': 1739, 'k': 4.113, 'v0': 1.126},
    2: {'rpm_max': 2956, 'k': 3.673, 'v0': 1.302},
    3: {'rpm_max': 4173, 'k': 3.673, 'v0': 1.445},
    4: {'rpm_max': 5389, 'k': 2.706, 'v0': 1.588},
    5: {'rpm_max': 6597, 'k': 2.337, 'v0': 1.760},
    6: {'rpm_max': 7960, 'k': 2.057, 'v0': 1.932}
}

# ==========================
# 2. VARIABLES GLOBALES PARTAGÉES
# ==========================
verrou = threading.Lock()
rapport_virtuel_pico = 1
mode_conduite = MODE_INIT
etat_systeme = ERR_OK
pico_serial = None
motor_pwm = None
rpm_reel = 0.0

# ==========================
# 3. FONCTIONS HARDWARE
# ==========================

def arret_urgence(channel):
    global mode_conduite, motor_pwm
    
    with verrou:
        mode_conduite=MODE_INIT
        if motor_pwm:
            motor_pwm.ChangeDutyCycle()

def setup_hardware():
    global motor_pwm
    GPIO.setup(PIN_MOTOR_PWM, GPIO.OUT)
    motor_pwm = GPIO.PWM(PIN_MOTOR_PWM, 100)
    motor_pwm.start(0)

def setup_serial():
    global pico_serial
    try:
        pico_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        pico_serial.flush()
        t = threading.Thread(target=ecouter_pico_background, daemon=True)
        t.start()
        print(f"CONNECTÉ SUR {SERIAL_PORT}")
    except:
        print(f"PAS DE PICO (MODE SOLO)")

def lire_pedale_brute_testee():
    global etat_systeme
    try:
        # 1. Lecture brute
        v1 = chan_pedale_1.voltage
        v2 = chan_pedale_2.voltage
        
        # 2. Sécurité physique (Fil coupé)
        # On vérifie si chaque capteur descend sous son propre seuil - 0.2V
        if v1 < (PEDALE_V_MIN_C1 - 0.2) or v2 < (PEDALE_V_MIN_C2 - 0.2):
            etat_systeme = ERR_CAP
            return 0.0
        
        # 3. Calcul des ratios (Normalisation)
        ratio_v1 = (v1 - PEDALE_V_MIN_C1) / (PEDALE_V_MAX_C1 - PEDALE_V_MIN_C1)
        ratio_v2 = (v2 - PEDALE_V_MIN_C2) / (PEDALE_V_MAX_C2 - PEDALE_V_MIN_C2)
        
        # 4. Saturation entre 0 et 1
        r1_sat = max(0.0, min(1.0, ratio_v1))
        r2_sat = max(0.0, min(1.0, ratio_v2))
        
        # 5. Vérification de la cohérence entre les deux pédales
        # On utilise une tolérance un peu plus large (0.2) pour éviter les fausses alertes
        if abs(r1_sat - r2_sat) > 0.3:
            etat_systeme = ERR_CAP
            return v1 # On renvoie v1 mais l'état dira CAP MIN
            
        # Si tout est arrivé ici sans planter
        etat_systeme = ERR_OK
        return v1

    except Exception as e:
        # Si ça plante encore, on affiche l'erreur réelle dans la console pour debugger
        print(f"DEBUG Erreur Interne: {e}")
        etat_systeme = ERR_ALL
        return 0.0

'''
    except:
        etat_systeme = ERR_ALL
        return 0.0
    
    # Sécurité
    v1 = max(0.0, v1)
    v2 = max(0.0, v2)
''' 
def calibrer_pedale(tension_brute):
    # Mapping linéaire (Pont diviseur -> 0..4.5V)
    if tension_brute < (PEDALE_V_MIN_REEL + PEDALE_ZONE_MORTE): return 0.0
    if tension_brute > PEDALE_V_MAX_REEL: return PEDALE_MAX_OUT
    
    plage = PEDALE_V_MAX_REEL - PEDALE_V_MIN_REEL
    if plage <= 0: return 0.0
    
    ratio = (tension_brute - PEDALE_V_MIN_REEL) / plage
    return max(0.0, min(PEDALE_MAX_OUT, ratio * PEDALE_MAX_OUT))

def envoyer_au_pico(rpm, rpm_max_rapport):
    if not pico_serial: return
    pct = 0
    if rpm_max_rapport > 0: pct = (rpm / rpm_max_rapport) * 100
    try:
        # Format du message validé par ton test Pico
        msg = f"{int(pct)},{etat_systeme}\r\n"
        print(msg)

        pico_serial.write(msg.encode('utf-8'))
    except: pass

# ==========================
# 4. THREAD ÉCOUTE PICO (Issu de ton Test)
# ==========================
def ecouter_pico_background():
    """
    C'est exactement la logique de ton 'Test Pico' qui décortique
    """
    global rapport_virtuel_pico, mode_conduite
    
    while True:
        if pico_serial and pico_serial.in_waiting > 0:
            try:
                # Lecture robuste (errors='replace') comme dans ton test
                line = pico_serial.readline().decode('utf-8', errors='replace').strip()
                ligne=line.lower().replace(" ","")
                if line:
                    print(f": '{ligne}'")
                    new_mode=-1
                    line_low = line.lower()
                    if "elec" in ligne:
                        with verrou: mode_conduite = MODE_AUTO
                    elif "simu" in ligne:
                        with verrou: mode_conduite = MODE_SIMU
                    elif "init" in ligne:
                        with verrou: mode_conduite = MODE_INIT
                    new_rapport=-1
                    if "r=" in ligne:
                        try:
                            parts=ligne.split("r=")
                            if len(parts)>1:
                                char_r=parts[1][0]
                                if char_r.isdigit():
                                    new_rapport=int(char_r)
                        except : pass
                    with verrou:
                        if new_mode !=-1:
                            mode_conduite = new_mode
                            print(f"Mode change : {mode_conduite}")
                        if new_rapport !=-1:
                            rapport_virtuel_pico=new_rapport
                            print(f"Rapport change : {rapport_virtuel_pico}")
            except Exception as e:
                pass
        time.sleep(0.01)

# ==========================
# 5. MATHÉMATIQUES MOTEUR
# ==========================
def calcul_rpm_manuel(pedale_v, rapport):
    if rapport not in Parametre_Rapport: return 0
    params = Parametre_Rapport[rapport]
    if pedale_v < 0.1: return 0
    exp = -params['k'] * (pedale_v - params['v0'])
    return params['rpm_max'] / (1 + math.exp(exp))

def calcul_voltage_inverse(target_rpm, rapport):
    if rapport not in Parametre_Rapport: return 0
    if target_rpm <= 100: return 0
    params = Parametre_Rapport[rapport]
    if target_rpm >= params['rpm_max'] * 0.99: return PEDALE_MAX_OUT
    try:
        terme = (params['rpm_max'] / target_rpm) - 1
        if terme <= 0: return PEDALE_MAX_OUT
        vt = params['v0'] - (1/params['k']) * math.log(terme)
        return max(0.0, min(PEDALE_MAX_OUT, vt))
    except: return 0

# ==========================
# 6. BOUCLE PRINCIPALE
# ==========================
def main():
    global rpm_reel, rapport_virtuel_pico, mode_conduite
    
    setup_hardware()
    setup_serial()
    
    voltage_calcul_interne = 0.0
    dernier_rapport_connu = 1
    temps_depart=time.time()
    print("\n=== DÉMARRAGE DU SYSTÈME FINAL ===")
    
    try:
        while True:
            # 1. ACQUISITION
            temps_actuel=time.time()-temps_depart
            tension_brute = lire_pedale_brute_testee()
            #pedale_physique = calibrer_pedale(tension_brute)
            pedale_physique=(tension_brute-1.64)/(3.74-1.64)*4.5
            pedale_physique=max(0,min(4.5, pedale_physique))
            
            if pedale_physique<0.1:
                pedale_physique=0
            
            # MODE FORCÉ DRIVE
            levier = "D"
            
            target_rpm = 0
            rpm_max_aff = 8000
            rapport_actuel_aff="N"
            
            v1_debug=chan_pedale_1.voltage
            v2_debug=chan_pedale_2.voltage
            # 2. LOGIQUE
            if levier == "D":
                #print(f"debug mode actuel = {mode_conduite}")
                if mode_conduite == MODE_INIT:
                    target_rpm = 0
                    rmp_max_aff = 8000
                    rapport_actuel_aff= "N"
                    voltage_calcul_interne=pedale_physique
                
                elif mode_conduite == MODE_AUTO:
                    # Mode Auto : On suit le pied linéairement
                    if pedale_physique > 0.1: 
                        target_rpm = (pedale_physique / PEDALE_MAX_OUT) * RPM_MOTEUR_MAX
                    target_rpm = min(target_rpm, RPM_MOTEUR_MAX)
                    rpm_max_aff = RPM_MOTEUR_MAX
                    voltage_calcul_interne = pedale_physique
                    rapport_actuel_aff = "A"
                    
                elif mode_conduite == MODE_SIMU:
                    # Mode F1 : Gestion Inertie + Rapports
                    rapport_utilise = rapport_virtuel_pico
                    if rapport_utilise not in Parametre_Rapport:
                        rapport_utilise = 1
                    
                    # Continuité
                    if rapport_utilise != dernier_rapport_connu:
                        voltage_virtuel = calcul_voltage_inverse(rpm_reel, rapport_utilise)
                        voltage_calcul_interne = voltage_virtuel
                        dernier_rapport_connu = rapport_utilise
                    
                    # Inertie (Lissage 0.2 comme dans ton test)
                    ecart = pedale_physique - voltage_calcul_interne
                    vitesse_reprise = 0.2
                    
                    if abs(ecart) > 0.05:
                        voltage_calcul_interne += ecart * vitesse_reprise
                    else:
                        voltage_calcul_interne = pedale_physique
                        
                    target_rpm = calcul_rpm_manuel(voltage_calcul_interne, rapport_utilise)
                    rpm_max_aff = Parametre_Rapport[rapport_utilise]['rpm_max']
                    rapport_actuel_aff = rapport_utilise
            
            rpm_reel = target_rpm
            
            # 3. ACTION MOTEUR
            if RPM_MOTEUR_MAX > 0: duty = (target_rpm / RPM_MOTEUR_MAX) * 100
            else: duty = 0
            if motor_pwm: motor_pwm.ChangeDutyCycle(max(0, min(100, duty)))
            
            # 4. FEEDBACK PICO
            envoyer_au_pico(target_rpm, rpm_max_aff)
            
            # Affichage Debug
            print(f"Brut:{tension_brute:4.2f}V | Pied:{pedale_physique:4.2f}V | RPM:{int(target_rpm):4d} | Mode:{mode_conduite} R:{rapport_actuel_aff} | Etat:{etat_systeme}" , flush=True)
            time.sleep(0.02) # 50 Hz
            
    except KeyboardInterrupt:
        pass
    finally:
        print("Arrêt.")
        if motor_pwm: motor_pwm.stop()
        GPIO.cleanup()
        if pico_serial: pico_serial.close()

if __name__ == "__main__":
    main()
