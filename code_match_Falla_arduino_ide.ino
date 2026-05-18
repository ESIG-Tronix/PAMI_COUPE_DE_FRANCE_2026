//==========================================================================
//                                 LIBRAIRIES
//==========================================================================
 
 #include <Arduino.h>
 #include <DFRobot_VL6180X.h>
 #include <Wire.h>
 #include <ESP32Servo.h>
 
//==========================================================================
//                                 PINS
//==========================================================================
 
//Tirette
const int tirette = 21;
 
//Moteur A
const int PWMA = 14;
const int AIN1 = 25;
const int AIN2 = 27;
const int encoder1A = 26;
const int encoder2A = 13;
 
//Moteur B
const int PWMB = 32;
const int BIN1 = 5;
const int BIN2 = 19;
const int encoder1B = 22;
const int encoder2B = 23;
 
//Servo
const int servo_pin = 33;
 
//registre
const int SRCLK = 18;
const int SRCLR = 12;

//équipe
const int interrupteur = 15;
const int led = 2;

//==========================================================================
//                                 VARIABLES
//==========================================================================
 
//Odométrie et asservissement
long virageTicks = 650;
const float PPR = 1400.0;
const float DIAMETRE_MM = 38.0;
const float ENTRAXE_MM = 81; //81.98 officiel mais avec glissement etc => marge corrige grace a  ENTRAXE_MM = 80.4 * (180 / 160) = 90.45 et antoine a noté 86
const float MM_PAR_PULSE = (PI * DIAMETRE_MM) / PPR;
const float COMPENS_GAUCHE = 1.12; //0.925 ded base
volatile long pulseCount_D = 0; volatile long pulseCount_G = 0;
volatile unsigned long lastD = 0; volatile unsigned long lastG = 0;
long prev_pulses_D = 0; long prev_pulses_G = 0;
float pos_x = 0.0, pos_y = 0.0, pos_theta = 0.0;
 
//Servo
Servo myservo;
int angle = 0;
 
//Tof
DFRobot_VL6180X tof1(0x31, &Wire);
DFRobot_VL6180X tof2(0x30, &Wire);
DFRobot_VL6180X tof3(0x29, &Wire);
 
//Timer
unsigned long temps = 0;
const unsigned long delai = 85000;   //A CHANGER, METTRE 85000 POUR LE MATCH
const unsigned long match = 100000;

//Evitement
float dec1 = 0;
float dec2 = 0;

//destination
float dec3 = 0;
float dec4 = 0;

 
//==========================================================================
//                                 SWITCH CASE
//==========================================================================
 
//Définition RobotState
enum RobotState { ATTENTE, AVANCER, EVITE, FIN };
RobotState currentState = ATTENTE;
 
//==========================================================================
//                                 FONCTIONS
//==========================================================================
 
void horloge(){
    digitalWrite(SRCLK, HIGH);
    delay(10);
    digitalWrite(SRCLK, LOW);
    delay(10);
    Serial.println("COUP D'HORLOGE");
}
 
//Fonctions d'interruption
void IRAM_ATTR countPulse_D() {
    if (micros() - lastD < 300) return;
    if (digitalRead(encoder1A) == digitalRead(encoder2A)) pulseCount_D++; else pulseCount_D--;
    lastD = micros();
}
 
void IRAM_ATTR countPulse_G() {
    if (micros() - lastG < 300) return;
    if (digitalRead(encoder1B) == digitalRead(encoder2B)) pulseCount_G--; else pulseCount_G++;
    lastG = micros();
}
 
//Fonction setMoteur
void setMoteurs(float vit_D, float vit_G) {
    float vD = -vit_D;
    float vG = (vit_G * COMPENS_GAUCHE);
 
    if (abs(vD) > 1 && abs(vD) < 60) vD = (vD > 0) ? 60 : -60;
    if (abs(vG) > 1 && abs(vG) < 60) vG = (vG > 0) ? 60 : -60;
 
    vD = constrain(vD, -250, 250);
    vG = constrain(vG, -250, 250);
 
    digitalWrite(AIN1, vD >= 0 ? HIGH : LOW); digitalWrite(AIN2, vD >= 0 ? LOW : HIGH);
    ledcWrite(PWMA, abs((int)vD));
    digitalWrite(BIN1, vG >= 0 ? HIGH : LOW); digitalWrite(BIN2, vG >= 0 ? LOW : HIGH);
    ledcWrite(PWMB, abs((int)vG));
}
 
//Fonction mettreAJourPosition
void mettreAJourPosition() {
    noInterrupts();
    long cD = pulseCount_D; long cG = pulseCount_G;
    interrupts();
 
    float dD = (cD - prev_pulses_D) * MM_PAR_PULSE;
    float dG = (cG - prev_pulses_G) * MM_PAR_PULSE;
   
    if (abs(dD) > 20 || abs(dG) > 20) return;
    prev_pulses_D = cD; prev_pulses_G = cG;
 
    float dist = (dD + dG) / 2.0;
    float dTheta = (dD - dG) / ENTRAXE_MM;
    pos_theta += dTheta;
 
    while (pos_theta > PI) pos_theta -= 2 * PI;
    while (pos_theta < -PI) pos_theta += 2 * PI;
    pos_x += dist * cos(pos_theta);
    pos_y += dist * sin(pos_theta);
}
 
//Fonction goTo
bool goTo(float x_target, float y_target, bool reset = false) {
 
    static int phase = 1;
    //le if sert à reset et remettre phase=1 sinon le robot continue à se décaler sans s'arrêter
    if(reset){
        phase = 1;
        return false;
    }
    mettreAJourPosition();
   
    float dx = x_target - pos_x;
    float dy = y_target - pos_y;
    float dist = sqrt(dx*dx + dy*dy);
    float angle_cible = atan2(dy, dx);
    float err_a = angle_cible - pos_theta;
   Serial.printf("phase:%d dist:%.1f err_a:%.3f\n", phase, dist, err_a);
 
    while(err_a > PI) err_a -= 2*PI;
    while(err_a < -PI) err_a += 2*PI;
    //si suffisamment proche des coordonnées on dit qu'on est arrivé
    if (dist < 15.0) {
        setMoteurs(0,0);
        phase = 1;
        return true;
    }
   
    //si on s'oriente
    if (phase == 1) {
        //si on est bien orienté on passe au déplacement
        if (abs(err_a) < 0.05) {
            setMoteurs(0,0);
            delay(300);
            phase = 2;
            return false;
        }
        //sinon oncontinue à s'orienter
        float rot = constrain(err_a * 25.0, -250, 250);  // FIX : était 40
        setMoteurs(rot, -rot);
    }
    //si on se déplace
    else {
        float kp_angle = -230.0;  // FIX : était 35
        float corr = err_a * kp_angle;
       
        if (abs(err_a) > 0.5) { phase = 1; return false; }
 
        float vit_droit = 250 - corr;
        float vit_gauche = 250 * COMPENS_GAUCHE + corr;  // FIX précédent : était /COMPENS_GAUCHE
        setMoteurs(vit_droit, vit_gauche);
    }
    //tant qu'on est pas suffisamment près on continue et on ne s'arrête pas
    return false;
}
 
void testerServo(){
    for(angle = 0 ; angle < 180 ; angle +=3){
        myservo.write(angle);
        delay(15);
    }
    delay(500);
    for(angle = 180 ; angle >= 1 ; angle -=3){
        myservo.write(angle);
        delay(5);
    }
    delay(500);
}
 
//==========================================================================
//                                 SETUP
//==========================================================================
 
void setup() {
    Serial.begin(115200);
 
    //Servo
    myservo.attach(servo_pin);
 
    //Bouton
    pinMode(tirette, INPUT_PULLUP);

    //Equipe
    pinMode(led, OUTPUT);
    pinMode(interrupteur, INPUT_PULLUP);
 
    //Moteurs
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(encoder1A, INPUT_PULLUP); pinMode(encoder2A, INPUT_PULLUP);
    pinMode(encoder1B, INPUT_PULLUP); pinMode(encoder2B, INPUT_PULLUP);
 
    //Horloge
    pinMode(SRCLK , OUTPUT);
    pinMode(SRCLR , OUTPUT);
 
    digitalWrite(SRCLK , LOW);
    Wire.begin(17 , 16);
 
    //Reset
    Serial.println("Reset");
    digitalWrite(SRCLR, LOW);
    delay(100);
    digitalWrite(SRCLR, HIGH);
    delay(10);
    horloge();
    delay(100);  
 
    //adressage TOF1
    horloge();
    delay(200);
    DFRobot_VL6180X tof1_temp(0x29, &Wire);
    if(tof1_temp.begin()){
        tof1_temp.setIICAddr(0x31);
        delay(50);
        Serial.println("tof1 adresse");
        if(tof1.begin()) Serial.println("tof1 pret");
    } else {
        Serial.println("tof1 non adresse");
    }
 
    //adressage TOF2
    horloge();
    delay(200);
    DFRobot_VL6180X tof2_temp(0x29, &Wire);
    if(tof2_temp.begin()){
        tof2_temp.setIICAddr(0x30);
        delay(50);
        Serial.println("tof2 adresse");
        if(tof2.begin()) Serial.println("tof2 pret");
    } else {
        Serial.println("tof2 non adresse");
    }
 
    //adressage TOF3
    horloge();
    delay(200);
    DFRobot_VL6180X tof3_temp(0x29, &Wire);
    if(tof3_temp.begin()){
        Serial.println("tof3 adresse");
        if(tof3.begin()) Serial.println("tof3 pret");
    } else {
        Serial.println("tof3 non adresse");
    }
 
 
    ledcAttach(PWMA, 1000, 8); ledcAttach(PWMB, 1000, 8);
 
    attachInterrupt(digitalPinToInterrupt(encoder1A), countPulse_D, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder1B), countPulse_G, CHANGE);
}
 
//==========================================================================
//                                 LOOP
//==========================================================================
 
void loop() {
 
    uint8_t range1 = tof1.rangePollMeasurement();
    uint8_t range2 = tof2.rangePollMeasurement();
    uint8_t range3 = tof3.rangePollMeasurement();
    mettreAJourPosition();
 
    if (temps > 0 && millis() - temps >= match){
        setMoteurs(0,0);
        currentState = FIN;
    }
 
    switch(currentState) {
        case ATTENTE:{
            setMoteurs(0,0);

            if (digitalRead(tirette) == HIGH && temps == 0) {
                temps = millis();
                Serial.println("Timer démarré");
            }
            if (digitalRead(interrupteur) == HIGH){ //high = équipe bleue
                digitalWrite(2, HIGH);
                dec1 = -50;
                dec2 = -200;
                dec3 = -250; //A CHANGER AVANT MATCH
                dec4 = 200;
            }
            else{   //low = équipe jaune
                digitalWrite(2, LOW);
                dec1 = 50;
                dec2 = 200;
                dec3 = 250;    //A CHANGER AVANT MATCH
                dec4 = -200;
            }

            //verifier que la tirette est bien mise pour pas qu'il parte tout seul
            if (digitalRead(tirette) == LOW){
                digitalWrite(2, HIGH);
                delay(50);
                digitalWrite(2,LOW);
            }

            if (temps > 0 && millis() - temps >= delai){
                pulseCount_D = 0; pulseCount_G = 0;
                prev_pulses_D = 0; prev_pulses_G = 0;
                pos_x = 0; pos_y = 0; pos_theta = 0;
                Serial.println("Départ");
                currentState = AVANCER;
            }
            break;
        }
 
        case AVANCER:{
            static bool avancement_fixe = false;

            Serial.println("RETOUR ETAT AVANCER");

            if (!avancement_fixe){
                if(goTo(200,dec4)){    //C'EST POUR EVITER LE GRAND ROBOT, FAUT CREER UNE VARIABLE SELON L'EQUIPE
                    goTo(0,0,true);
                    avancement_fixe = true;
                }
            }
            else{
                float distance = sqrt(pow(800 - pos_x, 2) + pow(dec3 - pos_y, 2)); //A CHANGER AVANT MATCH EN FONCTIONS DES COORDONNEES DONNEES DANS LE goTo
                if (distance > 250){
                    if(range2 <= 45 && pos_x > 500){
                        //les 2 lignes ci-dessous permettent de gagner en précision car le robot va s'arrêter et remettre phase=1 (lié au if de la ligne 120)
                        setMoteurs(0,0);
                        goTo(0,0,true);
                        currentState = EVITE;
                        break;
                    }
                }
                if(goTo(800,dec3)){    //A CHANGER AVANT MATCH
                    avancement_fixe = false;
                    currentState = FIN;
                }
            }
            break;
        }
 
        case EVITE:{
            //variables qui vont permettre de faire le décalage
            static float evite_x0 = 0;   //x auquel on veut aller pour faire le premier décalage
            static float evite_y0 = 0;   //y auquel on veut aller pour faire le premier décalage
            static float evite_x1 = 0;   //x auquel on veut aller pour faire le deuxième décalage
            static float evite_y1 = 0;   //y auquel on veut aller pour faire le deuxième décalage
            static float evite_x2 = 0;   //x auquel on veut aller pour faire le deuxième décalage
            static float evite_y2 = 0;   //y auquel on veut aller pour faire le deuxième décalage
            static bool decalage_fixe = false;  //variable qui permet de fixer les coordonnées qu'on veut pour faire le décalage d'éviter de recalculer en boucle les 2 variables ci-dessus
            static int etape_evite = 0; //variable qui permet de savoir si on fait le premier ou le deuxième décalage
 
            Serial.println("RETOUR ETAT EVITE");
            Serial.printf("M1:%ld M2:%ld\n", pulseCount_D, pulseCount_G);
 
            if (!decalage_fixe){    //si décalage pas fixé
                setMoteurs(0,0);
                //on calcule les coordonnées qu'on veut pour le premier décalage
                evite_x0 = pos_x;
                evite_y0 = pos_y + dec1;
                //on calcule les coordonnées qu'on veut pour le deuxième décalage
                evite_x1 = evite_x0;
                evite_y1 = evite_y0 + dec2;
                //on calcule les coordonnées qu'on veut pour le deuxième décalage
                evite_x2 = evite_x1+200;
                evite_y2 = evite_y1;
                decalage_fixe = false;
                goTo(0,0,true); //on reset goTo car on va utiliser la fonction pour faire le décalage à la ligne 316
                decalage_fixe = true;   //on fixe les coordonnées qu'on veut pour le décalage
                Serial.printf("etape 1: X1:%.1f Y1:%.1f | etape 2: X2:%.1f Y2:%.1f\n", evite_x1, evite_y1,evite_x2,evite_y2);
            }

            //étape 0
            if (etape_evite == 0){
                if (goTo(evite_x0,evite_y0)){
                        Serial.println("Etape 0 faite");
                        goTo(0,0,true);
                        etape_evite = 1;
                    }
            }
 
            //étape 1
            if (etape_evite == 1){
                if(range1 <= 45 || range2 <= 45 || range3 <= 45){
                    //les 2 lignes ci-dessous permettent de gagner en précision car le robot va s'arrêter et remettre phase=1 (lié au if de la ligne 120)
                    setMoteurs(0,0);
                    goTo(0,0,true);
                    decalage_fixe = false;
                    etape_evite = 0;
                }
                else{
                    if (goTo(evite_x1,evite_y1)){
                        Serial.println("Etape 1 faite");
                        goTo(0,0,true);
                        etape_evite = 2;
                    }
                }
            }
 
            //étape 2
            else if (etape_evite == 2){
                if(range1 <= 45 || range2 <= 45 || range3 <= 45){
                    //les 2 lignes ci-dessous permettent de gagner en précision car le robot va s'arrêter et remettre phase=1 (lié au if de la ligne 120)
                    setMoteurs(0,0);
                    goTo(0,0,true);
                    decalage_fixe = false;
                    etape_evite = 0;
                }
                else{
                    if (goTo(evite_x2,evite_y2)){
                        Serial.println("Etape 2 faite");
                        Serial.println("Esquive faite");
                        decalage_fixe = false;
                        etape_evite = 0;
                        goTo(0,0,true);
                        currentState = AVANCER;
                    }
                }
            }
            break;
        }
 
        case FIN:{
            setMoteurs(0,0);
            testerServo();
            break;
        }
       
 
    }
    delay(10);
}
