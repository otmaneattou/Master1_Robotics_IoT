//include des librairies 
#include <Servo.h> // librairie servomoteurs
#include <SPI.h> // librairie du doute
#include <time.h> // librairie temps 
//**********definition des differentes parties des pattes du robot
#define coudeavantdroite 0
#define coudeavantgauche 1
#define coudearrieregauche 2
#define coudearrieredroite 3
#define epauleavantdroite 4
#define epauleavantgauche 5
#define epaulearrieregauche 6
#define epaulearrieredroite 7
#define jambehautdroite 8
#define jambehautgauche 9
#define jambebasgauche 10
#define jambebasdroite 11
//**********definition des positions des pattes
#define avantdroite 20
#define avantgauche 21
#define arrieredroite 22
#define arrieregauche 23
//**********definition des constantes 
const float pi = 3.1415;
const float L0 = 16.4; //longueur de la base
const float L1 = 5.1;//longueur de
const float L2 = 5; // longueur de 
// ****************angles utilisé pour les calcules*******************
float angle[]={0,0};
float angle1[]={0,0};
float angle2[] = {0,0};
typedef struct Patte Patte;
//********************************CREATION STRUCTURE PATTE**********************************
struct Patte 
{
  int id;
  int coude;
  int epaule;
  float angleepaule;
  float anglecoude;
};
//******************************* declaration des pattes********************************************
Patte patteavantgauche;
Patte patteavantdroite;
Patte pattearrieregauche;
Patte pattearrieredroite;
//**************************************VARIABLES SERVOMOTEUR********************************************
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;
Servo servo9;
Servo servo10;
Servo servo11;

//************************************************SETUP **********************************************/
void setup() {

  delay(200); // Pause de 200 millisecondes
  
  Serial.begin(9600); //Initialisation de la communication série

  // Assignation des ports servomteurs (Vue de face : fil de communication vers nous, là où il y a les yeux)
  servo0.attach(0); // coude avant droit
  servo1.attach(1); // coude avant gauche
  servo2.attach(2); // coude arrière gauche
  servo3.attach(3); // coude arrière droit
  servo4.attach(4); // épaule avant droit
  servo5.attach(5); // épaule avant gauche
  servo6.attach(6); // épaule arrière gauche
  servo7.attach(7); // épaule arrière droite
  servo8.attach(18); // jambe haut droite
  servo9.attach(10); // jambe haut gauche
  servo10.attach(8); // jambe bas gauche
  servo11.attach(19); // jambe bas droite
// definir les attributs des pattes
patteavantgauche.id = avantgauche;
patteavantgauche.coude = coudeavantgauche;
patteavantgauche.epaule = epauleavantgauche;


patteavantdroite.id = avantdroite;
patteavantdroite.coude = coudeavantdroite;
patteavantdroite.epaule = epauleavantdroite;


pattearrieregauche.id = arrieregauche;
pattearrieregauche.coude = coudearrieregauche;
pattearrieregauche.epaule = epaulearrieregauche;


pattearrieredroite.id = arrieredroite;
pattearrieredroite.coude = coudearrieredroite;
pattearrieredroite.epaule = epaulearrieredroite;

// mettre le robot dans sa position initiale( debout)
delay(15000);
  initialiser();
  delay(2000);
}
//***********************************BOUCLE PRINCIPALE***************************
void loop() {
 initialiser();
 assis();
 initialiser();
 lever(patteavangtgauche,pattearrieredroite)
 //marcher();
//calibrer(jambebasgauche,90);

}
//************fonction utilisé pour calibrer un servo***********
void calibrer(int moteur,int initiale)
{
   CommandeMoteur(moteur,initiale);
   delay(2000);
   for (int i=0; i<180; i+=5)
   {
      CommandeMoteur(moteur,initiale-i);
      delay(3000);
   }
}
//**********************BOUGER UNE PATTE**********************************
void bougerpatte(Patte* patte, float angle[])
{
  if(patte->id == avantgauche || patte->id == arrieredroite)
  {
    patte->angleepaule=angle[0];
    patte->anglecoude=angle[1];
    CommandeMoteur(patte->epaule,angle[0]);
    CommandeMoteur(patte->coude,angle[1]);
  }
  else
   {
    patte->angleepaule=angle[0];
    patte->anglecoude=angle[1];
    CommandeMoteur(patte->epaule,180-angle[0]);
    CommandeMoteur(patte->coude,180-angle[1]);
  }
}
//***************************COMMANDE UNE SERVOMOTEUR**************************
void CommandeMoteur(int moteur, int angle){
  
  Servo servo;

  switch (moteur){
    case 0:
      servo = servo0;
      break;
    case 1:
      servo = servo1;
      break;
    case 2:
      servo = servo2;
      break;
    case 3:
      servo = servo3;
      break;
    case 4:
      servo = servo4;
      break;
    case 5:
      servo = servo5;
      break;
    case 6:
      servo = servo6;
      break;
    case 7:
      servo = servo7;
      break;
    case 8:
      servo = servo8;
      break;
    case 9:
      servo = servo9;
      break;
    case 10:
      servo = servo10;
      break;
    case 11:
      servo = servo11;
      break;
    }

    Pente(servo,angle);
    //servo.write(angle);
  }
//***************************REGLER VITESSE SERVOMOTEUR****************************
void Pente(Servo servo, int consigne_angle){
  
  int previous_angle = servo.read();
  int angle = previous_angle;
  int interval = 20;

  if (previous_angle < consigne_angle){
    while(angle < consigne_angle){
      delay(interval);
      angle += 5;
      angle = constrain(angle,previous_angle,consigne_angle);
      servo.write(angle);
    }
  }
  else{
    while(angle > consigne_angle){
      delay(interval);
      angle -= 5;
      angle = constrain(angle,consigne_angle,previous_angle);
      servo.write(angle);
    }
  }

}


//*******************************CONVERSION DES ANGLES*********************************
float radtodeg(float angle)
{
  return (angle*180)/pi;
}
float degtorad(float angle)
{
  return (angle*pi)/180;
}

//******************************AVANCER UNE PATTE***********************************
void avancer (Patte patte)
{
  angle[0] = patte.angleepaule;
  angle[1] = patte.anglecoude;
  float d_base_sol;
  float d_leve;
  d_base_sol = L1*sin(degtorad(angle[0]))+L2*sin(degtorad(angle[1]-angle[0]));
  d_leve = L1*sin(degtorad(angle[0]));

//  angle[1]=radtodeg(pi-(asin((d_leve-L1*sin(degtorad(angle[0])))/L2)+degtorad(angle[0])));// lever la patte 
  if ( patte.id==arrieregauche||patte.id==arrieredroite)
  angle[1]=45;
  else 
  angle[1]= 20;
  bougerpatte(&patte,angle); 
  
//  delay(1000);
  if (patte.id==arrieregauche || patte.id == arrieredroite)
    angle[0]-=5*5;
  else
    angle[0]+=5*5;
  angle[1]=radtodeg(pi-(asin((d_leve-L1*sin(degtorad(angle[0])))/L2)+degtorad(angle[0])));
// avancer la patte en le maintenant levé
  bougerpatte(&patte,angle);
  
//  delay (1000);
if(patte.id == arrieregauche||patte.id == arrieredroite)
{
  angle[0] = radtodeg(asin((d_base_sol-L2)/L1))+10;
  angle[1] = 90+angle[0];
} 

else
  angle[1] = radtodeg(pi-(asin((d_base_sol-L1*sin(degtorad(angle[0])))/L2)+degtorad(angle[0])));
  bougerpatte(&patte,angle); //baisser la patte
//  delay (1000);
}
//**********************rculer**************
void reculer(Patte patte)
{ 
  angle[0]=45;
  angle[1]=90;
  bougerpatte(&patte,angle);
}
//**********************************ASSIS*******************************************
void assis()
{
  angle[0] = 0;
  angle[1] = 0;
  synchroniser(&patteavantgauche,&patteavantdroite,angle,angle,10);
}

//***********************************INITIALISER LE ROBOT***********************************
void initialiser()
{
   angle[0]=45;
   angle[1]=90;
   CommandeMoteur(jambehautdroite,70);
   CommandeMoteur(jambehautgauche,95);
   CommandeMoteur(jambebasdroite,82.5);
   CommandeMoteur(jambebasgauche,90);
  
   synchroniser(&patteavantgauche,&pattearrieredroite,angle,angle,25);
   synchroniser(&pattearrieregauche,&patteavantdroite,angle,angle,25);
}
//*********************************MARCHER*************************
void marcher()
{
  lever(patteavantdroite,pattearrieregauche);
  pousser(patteavantgauche,pattearrieredroite);
  poser(patteavantdroite,pattearrieregauche);
  ///delay(200);
  lever(patteavantgauche,pattearrieredroite);
  pousser(patteavantdroite,pattearrieregauche);
  poser(patteavantgauche,pattearrieredroite);
  //delay(200);
}
//********************************LEVER DEUX PATTES EN MEME TEMPS****************************
void lever(Patte patteAv, Patte patteAr)
{
  angle[0] = patteAv.angleepaule;
  angle1[0] = patteAr.angleepaule;
  angle1[1]=45;
  angle[1]=20;
  synchroniser(&patteAv,&patteAr,angle,angle1,10);
}
//********************************AVANCE DEUX PATTES EN MEME TEMPS***************************
void avancesynchrone(Patte patteAv, Patte patteAr)
{
  angle[0] = patteAv.angleepaule;
  angle[1] = patteAv.anglecoude;
  angle1[0] = patteAr.angleepaule;
  angle1[1] = patteAr.anglecoude;
  float d_base_sol;
  float d_leve;
  d_base_sol = L1*sin(degtorad(angle[0]))+L2*sin(degtorad(angle[1]-angle[0]));
  d_leve = L1*sin(degtorad(angle[0]));

  angle1[1]=45; 
  angle[1]= 20;
  synchroniser(&patteAv, &patteAr,angle,angle1,5);
  
  angle1[0]-=5*5;
  angle[0]+=5*5;
  synchroniser(&patteAv, &patteAr,angle,angle1,5);
  

  angle1[0] = radtodeg(asin((d_base_sol-L2)/L1));
  angle1[1] = 90+angle1[0];
  angle1[0]+=5;
  angle[1] = radtodeg(pi-(asin((d_base_sol-L1*sin(degtorad(angle[0])))/L2)+degtorad(angle[0])))+5;
  synchroniser(&patteAv, &patteAr,angle,angle1,5);
}
//***********************************AVANCER ET POSER DEUX PATTES LEVES PRECEDEMENT***************************
void poser(Patte patteAv, Patte patteAr)
{
  angle[1] = patteAv.anglecoude;
  angle1[1] = patteAr.anglecoude;
  float d_base_sol;
  d_base_sol = L1*sin(degtorad(45))+L2*sin(degtorad(45));

  angle1[0]=patteAr.angleepaule-5*5;
  angle[0]=patteAv.angleepaule+5*5;
  synchroniser(&patteAv, &patteAr,angle,angle1,5);

  angle1[0] = radtodeg(asin((d_base_sol-L2)/L1));
  angle1[1] = 90+angle1[0];
  angle1[0]+=10;
  angle[1] = radtodeg(pi-(asin((d_base_sol-L1*sin(degtorad(angle[0])))/L2)+degtorad(angle[0])))+20;
  synchroniser(&patteAv, &patteAr,angle,angle1,5);
}
//**********************************POUSSER LE ROBOT EN AVANT AVEC DEUX PATTES******************************
void pousser(Patte patteAv, Patte patteAr)
{ 
  angle[0]=45;
  angle[1]=90;
  angle1[0]=45;
  angle1[1]=90;
  synchroniser(&patteAv, &patteAr,angle,angle1,10);
}
//*******************************FAIRE BOUGER DEUX PATTES EN MEME TEMPS*******************************
void synchroniser(Patte* patte1,Patte* patte2,float angle[],float angle1[],int interval)
{
  Servo coude1 = choisirservo(patte1->coude);
  Servo epaule1= choisirservo(patte1->epaule);
  Servo coude2= choisirservo(patte2->coude);
  Servo epaule2= choisirservo(patte2->epaule);
  float previous_coude2 = coude2.read();
  float previous_coude1 = coude1.read();
  float previous_epaule2 = epaule2.read();
  float previous_epaule1 = epaule1.read();

  patte1->angleepaule = angle[0];
  patte2->angleepaule = angle1[0];
  patte1->anglecoude = angle[1];
  patte2->anglecoude = angle1[1];
  
  float consigne_angle[] = { angle[0], angle[1], angle1[0], angle1[1]};

  if (patte1->id == avantdroite || patte1->id == arrieregauche)
  {
     consigne_angle[0] = 180-patte1->angleepaule;
     consigne_angle[1] = 180-patte1->anglecoude;
  } 
  if (patte2->id == avantdroite || patte2->id == arrieregauche)
  {
     consigne_angle[2] = 180-patte2->angleepaule;
     consigne_angle[3] = 180-patte2->anglecoude;
  }
  
  float previous_angle[] = {previous_epaule1,previous_coude1,previous_epaule2,previous_coude2};
  Servo servo[] = {epaule1,coude1,epaule2,coude2} ;
  float Angle[] = {previous_epaule1,previous_coude1,previous_epaule2,previous_coude2};
  
  int i = 0;
  while(Angle[0]!=consigne_angle[0] || Angle[1] != consigne_angle[1] || Angle[2] != consigne_angle[2] || Angle[3] != consigne_angle[3])
  {
    for (i=0; i!=4; i++)
    {
        if (previous_angle[i] < consigne_angle[i])
        {
          delay(interval);
          Angle[i] += 5;
          Angle[i] = constrain(Angle[i],previous_angle[i],consigne_angle[i]);
          servo[i].write(Angle[i]);
        }
        else
        {
          delay(interval);
          Angle[i] -= 5;
          Angle[i] = constrain(Angle[i],consigne_angle[i],previous_angle[i]);
          servo[i].write(Angle[i]);
        }
    }
  }
}
//*******************************IDENTIIER UN SERVO PAR LE NOM DE L'ARTICULATION*************************
Servo choisirservo(int moteur)
{
  Servo servo;

  switch (moteur){
    case 0:
      servo = servo0;
      break;
    case 1:
      servo = servo1;
      break;
    case 2:
      servo = servo2;
      break;
    case 3:
      servo = servo3;
      break;
    case 4:
      servo = servo4;
      break;
    case 5:
      servo = servo5;
      break;
    case 6:
      servo = servo6;
      break;
    case 7:
      servo = servo7;
      break;
    case 8:
      servo = servo8;
      break;
    case 9:
      servo = servo9;
      break;
    case 10:
      servo = servo10;
      break;
    case 11:
      servo = servo11;
      break;
    }
    return servo;
}
