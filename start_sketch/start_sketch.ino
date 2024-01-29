#define LIGHT_SENSOR_PIN A0

#define PROX_SENSOR_L_PIN 6 //A1
#define PROX_SENSOR_R_PIN 9 //A2
#define PROX_SENSOR_FL_PIN A3
#define PROX_SENSOR_FR_PIN A4
#define PROX_SENSOR_RL_PIN A5
#define PROX_SENSOR_RR_PIN 12
#define PROX_SENSOR_DL_PIN A2 //6
#define PROX_SENSOR_DR_PIN A1 //9

#define MOTOR_RF_PIN 2
#define MOTOR_RB_PIN 4
#define MOTOR_R_SPEED 3
#define MOTOR_LF_PIN 7
#define MOTOR_LB_PIN 8
#define MOTOR_L_SPEED 5

//DEPLACEMENT
int distgauche=1023;
int distDgauche=1023;
int distfaceG=1023;
int const vit=100;
int distdroite=1023;
int distDdroite=1023;
int distfaceD=1023;
//Longe le mur de gauche
int const seuil1=500; //Petit seuil gauche
int const seuil2=700; // Grand seuil gauche
int const seuil3=450; //Pour en face
//Orientation du robot
enum Action {LIGNE, COIN, VIRAGE};
Action etat1=LIGNE;
Action etat2=LIGNE;
bool boolVIRAGE=false;

//REPERAGE DES COULEURS
enum Couleur {BLANC, ROUGE, NOIR, INCONNUE};
Couleur couleur_case1=INCONNUE;
Couleur couleur_case2=INCONNUE;
uint16_t mesure_couleur1 = 500;
uint16_t mesure_couleur2 = 500;
int const seuilROUGEhaut = 300;
int const seuilROUGEbas = 250;
int const seuilBLANC = 900;
int const seuilNOIR = 200;
//Détection case ou ligne
Couleur couleur_caseAvant=INCONNUE;
Couleur couleur_caseApres=INCONNUE;
int temps_couleur1=0;
int temps_couleur2=0;
int temps_couleur_case=0;
int const seuil_temps_case_ligne=300;
enum Quadrillage {LIGNE_NOIR, CASE_NOIR, CASE_BLANC, CASE_ROUGE};
Quadrillage const INIT_Objet_dessous = CASE_BLANC;
Quadrillage Objet_dessous1 = INIT_Objet_dessous;
Quadrillage Objet_dessous2 = INIT_Objet_dessous;

//Orientation labyrinthe
uint8_t const NligneLaby = 8; //////////////////////////////////////////
uint8_t const NcolonneLaby = 9; ////////////////////////////////////////
Couleur Labyrinthe[NligneLaby][NcolonneLaby]={INCONNUE};
uint8_t const INIT_robotX = 4; ////////////////////////////////////////////////////
uint8_t const INIT_robotY = 8; ////////////////////////////////////////////////////
uint8_t robotX = INIT_robotX;
uint8_t robotY = INIT_robotY;
uint8_t cursAffX =0;
uint8_t cursAffY =0;
enum Orientation {HAUT, DROITE, BAS, GAUCHE};
Orientation const INIT_Sens = GAUCHE; /////////////////////////////////////////////
Orientation Sens = INIT_Sens;
uint8_t SensEnChiffre = 0;
//LabyMure
uint8_t const NligneLabyMur = 2*(NligneLaby)+1;
uint8_t const NcolonneLabyMur = 2*(NcolonneLaby)+1;
enum Objet_LabyMur {INCONNU, MUR, PAS_MUR};
Objet_LabyMur LabyMur[NligneLabyMur][NcolonneLabyMur]={INCONNU};
uint8_t robotXMur = 2*(robotX+1)-1;
uint8_t robotYMur = 2*(robotY+1)-1;


//Strategie
enum Etape {TOUR, CHERCHE_NOIR, CHERCHE_ROUGE, ALEATOIRE};
Etape Phase=TOUR;
int Temps_Depart=0;
bool boolTempsNoir=false;
int Temps_Noir=0;
int Temps_Tour=0;
bool boolCaseRouge=true;
Orientation Choix_Strategique=HAUT;
bool Demi_Tour=false;
bool fin=false;
bool Case_Noir_A=false;
bool Case_Rouge_A=false;

//Aleatoire
int const vitesse_min_A=80;
int const vitesse_max_A=200;

//Initialisation
int const Temps_Initialisation = 5000;
bool Depart =false;

//Affichage
int Val1 =0;
int Val2 =0;
int Val3 =0;
int Val4 =0;
int Val5 =0;
int Val6 =0;

void hardware_setup() {
  new DCMotor_Hbridge(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_R_SPEED, "ePuck_rightJoint", 2.5, 3 * 3.14159, 1);
  new DCMotor_Hbridge(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_L_SPEED, "ePuck_leftJoint", 2.5, 3 * 3.14159, 1);

  new VisionSensor(LIGHT_SENSOR_PIN, "ePuck_lightSensor", 0.1);

  new ProximitySensor(PROX_SENSOR_FL_PIN, "ePuck_proxSensor3", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_FR_PIN, "ePuck_proxSensor4", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_L_PIN, "ePuck_proxSensor1", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_R_PIN, "ePuck_proxSensor6", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RL_PIN, "ePuck_proxSensor7", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RR_PIN, "ePuck_proxSensor8", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DL_PIN, "ePuck_proxSensor2", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DR_PIN, "ePuck_proxSensor5", 0.1, 1);
}

void setup() {
  Serial.begin(4800);

  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(MOTOR_R_SPEED, OUTPUT);
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_L_SPEED, OUTPUT);

  // Set speed to max
  analogWrite(MOTOR_R_SPEED, vit);
  analogWrite(MOTOR_L_SPEED, vit);

  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);

  MStop();
  Initialyse_Labyrinthe();
}

void loop() {
  //Serial.println("tic");
  if(millis()<Temps_Initialisation)// Pour éviter tout problème au démarage, le robot ne démare 
  {                                //qu'au bout de 5 secondes et les variables de départ sont rappelées
    Initialisation();//(voir ligne 464)
    //Serial.println("INITIALISE");
    Depart=false; //Le booléen "Depart" permet de bloquer les moteurs, le robot ne bouge pas
    Temps_Depart=millis(); //Il lance un chrono quand il part
  }
  else
  {
    Depart=true;
  }

  STRATEGIE_1();
}

//La strategie n°1 est assez classique, le robot suit uniquement le bord, en espérant que la case noir y soit
//si il trouve la case noir, il fini son tour, puis il fait l'aller-retour de la case rouge à la noir avec la direction la plus courte.
//si elle n'y est pas, il parcour aléatoirement le labyrinthe jusqu'à être passer sur la case noir et la rouge
void STRATEGIE_1()
{
  if(Phase==TOUR) //Premiere étape, faire le tour du labyrinthe en longeant le mur gauche et scanner chaque case en la répertoriant dans une simulation du labyrinthe
  {
    Construire_Tour_Labyrinthe(); // Cette fonction permet de construire la simulation
    if((Objet_dessous2==CASE_NOIR)&&(boolTempsNoir==false)) //Le robot note à quel instant il franchie la case noir
    {
      Temps_Noir=millis();
      boolTempsNoir=true;
    }
    if(Objet_dessous2==LIGNE_NOIR) // Cette condition permet au robot de ne pas prendre en compte la case rouge au départ du parcoure
    {
      boolCaseRouge=false;
    }
    if((Objet_dessous2==CASE_ROUGE)&&(boolCaseRouge==false)) //Si ce n'est pas la premiere fois que le robot passe par la case rouge
    {
      if((robotX==INIT_robotX)&&(robotY==INIT_robotY-1)&&(Sens==GAUCHE)) //On regarde que les cooredonnées sont bien les mêmes qu'au départ
      {
        Serial.println("Le tour du labyrinthe a été fait correctement");
        Temps_Tour=millis(); // Il compte combien il a mis de temps pour faire un tout complet
        if(Temps_Noir!=0) // et si il a trouver la case noir sur le bord
        {
          Phase=CHERCHE_NOIR; // il passe à l'étape suivante
        }
        else
        {
          Serial.println("Je n'ai pas trouvé la case noire pendant le tour, il faut aller chercher au milieu du labyrinthe"); //sinon il ne fait rien et tourne en rond
          Phase=ALEATOIRE;
        }
      }
      else //si les coordonnées ont changées, celà veut dire qu'il c'est trompé, il recommence
      {
        Initialisation2();
        Serial.println("Je me suis trompé, je recommence.");
        boolCaseRouge==true;
      }
    }
  }
  if(Phase==CHERCHE_NOIR) //La seconde étape consiste à aller à la case noir par le coté le plus court
  {
    if(Temps_Noir-Temps_Depart<(Temps_Tour-Temps_Depart)/2) // il regarde si la case est dans la premiere ou seconde moitié du parcoure..
    {
      Choix_Strategique=GAUCHE;//..et choisie ainsi la meilleur direction à prendre
      Serial.println("Le chemin est plus court si je longe le mur de gauche");
    }
    else
    {
      Choix_Strategique=DROITE;
      Serial.println("Le chemin est plus court si je longe le mur de droite");
    }
    if((Choix_Strategique==DROITE)&&(Demi_Tour==false)); //il se retourne si il n'est pas dans le bon sens
    {
      do
      {
        TGauche();
        distgauche=analogRead(PROX_SENSOR_L_PIN);
      } while(distgauche>seuil2);
      Demi_Tour=true;
    }
    do //puis suit le mur jusqu'à la case noir
    {
      if(Choix_Strategique==GAUCHE)
      {
        Longe_mur_gauche();
      }
      if(Choix_Strategique==DROITE)
      {
        Longe_mur_droite();
      }
      Voir_Quadrillage();
    } while(Objet_dessous2!=CASE_NOIR);
    Serial.println("J'ai trouvé la case noire, je retourner à la case rouge.");
    //il se retourne pour repartir
    if(etat2==VIRAGE)//sauf si il est dans un virage, car il a du mal à le faire dans ce cas là
    {
      do//alors il continue à longer le mur
      {
        if(Choix_Strategique==GAUCHE)
        {
          Longe_mur_gauche();
        }
        if(Choix_Strategique==DROITE)
        {
          Longe_mur_droite();
        }
        Voir_Quadrillage();
      } while(etat2==VIRAGE);
    }
    //Le sens dans lequel il se retourne dépend du sens dans lequel il est arrivé
    if(Choix_Strategique==GAUCHE)
    {
      do
      {
        TDroite();
        distDdroite=analogRead(PROX_SENSOR_DR_PIN);
      } while(distDdroite>seuil2);
    }
    if(Choix_Strategique==DROITE)
    {
      do
      {
        TGauche();
        distDgauche=analogRead(PROX_SENSOR_DL_PIN);
      } while(distDgauche>seuil2);
    }
    Phase=CHERCHE_ROUGE; // et passe à l'étape suivante
  }
  if(Phase==CHERCHE_ROUGE) //La dernière, il retourne à la case noir en longeant le mur
  {
    //il suit le mur dans l'autre sens pour revenir le plus vite possible à la case rouge
    do
    {
      if(Choix_Strategique==GAUCHE)
      {
        Longe_mur_droite();
      }
      if(Choix_Strategique==DROITE)
      {
        Longe_mur_gauche();
      }
      Voir_Quadrillage();
    } while(Objet_dessous2!=CASE_ROUGE); //une fois arrivé à la case rouge...
    //...il a fini et s'arrete pour toujours
    Serial.println("Fin");
    do
    {
      MStop();
    } while(fin==false);
  }
  if(Phase==ALEATOIRE)
  {
    Bouge_Aleatoire();
    Voir_Quadrillage();
    if(Objet_dessous2==CASE_NOIR)
    {
      Case_Noir_A=true;
    }
    if(Case_Noir_A==true)
    {
      if(Objet_dessous2==CASE_ROUGE)
      {
        Case_Rouge_A=true;
      }
    }
    if(Case_Rouge_A==true)
    {
      Serial.println("Retour a la case rouge");
      Serial.println("Fin");
      do
      {
        MStop();
      } while(fin==false);
    }
  }
}

//Cette fonction permet de déplacer le robot aléatoirement dans le labyrinthe, il peut ainsi parcourir le milieu de celui-ci
//Il utilise directement les capeurs de distance et pilote les moteurs grâce aux fonctions de déplacement qui lui sont propres (voir lignes 999 à 1027)
void Bouge_Aleatoire()
{
  distfaceG=analogRead(PROX_SENSOR_FL_PIN);
  distDgauche=analogRead(PROX_SENSOR_DL_PIN);
  distfaceD=analogRead(PROX_SENSOR_FR_PIN);
  distDdroite=analogRead(PROX_SENSOR_DR_PIN);
  if((distfaceG>500)&&(distfaceD>500)) //si n'y a rien devant, on va devant
  {
    Avancer_Aleatoire();
  }
  else //s'il y a un obstacle devant
  {
    if((distDgauche<500)||(distfaceG<500)) //si cet obstacle est à gauche, on tourne à droite
    {
      Avancer_Aleatoire();
      Tourner_Droite_Aleatoire();
    }
    if((distDdroite<500)||(distfaceD<500)) //si cet obstacle est à droite, on tourne à gauche
    {
      Avancer_Aleatoire();
      Tourner_Gauche_Aleatoire();
    }
  }
}

//Cette fonction permet de construire une simulation des couleurs des cases du labyrinthe et de la mémoriser (voit ligne 59 à 77 pour les variables)
//Le tableau "Labyrinthe" (ligne 62) est compléter au fure et à mesure que le robot longe le mur de gauche
//La fonction utilise les réponses des deux fonctions "Longe_mur_gauche" via les variables "etatX" et la réponse
// à la fonction "Voir_Quadrillage" via les variables "Objet_dessousX"
//Il y a aussi le tableau "LabyMur" (ligne 76) qui indique la position des murs dans le labyrinthe. Il est également compléter en direct.
//Cette fonction est le coeur de notre travail, elle est aussi très complexe, c'est pour cela qu'il y a un fichier pour décrire son fonctionnement.
void Construire_Tour_Labyrinthe()
{
  Longe_mur_gauche();
  Voir_Quadrillage();
  
  //Affichage de la direction et de la position du robot
  //PrintQueQuandCaChangeSens();
  //PrintQueQuandCaChangeCoordonnees();
  
  //Pour être en intéraction avec les simulations du Labyrinthe, plusieurs petites fonctions ont été créées pour
  // se déplacer, entrer les variables.... (voir lignes 487 à 683)
  if((etat2==COIN)&&(etat2!=etat1)) //si le robot est dans un coin c'est qu'il va tourner à droite
  {
    Tourne_Droite_Labyrinthe();
  }
  if(etat2==LIGNE) //si le robot va tout droit, on compte le nombre de cases qu'il parcoure pour connaitre la distance parcourue
  {
    if((Objet_dessous2==CASE_BLANC)&&(Objet_dessous2!=Objet_dessous1))
    {
      Labyrinthe[robotX][robotY]=BLANC; //Au passage, on rentre la couleur de la case dans la simulation
      PasMur_en_face_LabyMur(); //On note aussi dans le Labymur, qu'il n'y a pas de mur à cette endrois
      Avance_Labyrinthe();
    }
    if((Objet_dessous2==CASE_ROUGE)&&(Objet_dessous2!=Objet_dessous1))
    {
      Labyrinthe[robotX][robotY]=ROUGE;
      PasMur_en_face_LabyMur();
      Avance_Labyrinthe();
    }
    if((Objet_dessous2==CASE_NOIR)&&(Objet_dessous2!=Objet_dessous1))
    {
      Labyrinthe[robotX][robotY]=NOIR;
      PasMur_en_face_LabyMur();
      Avance_Labyrinthe();
    }
    Mur_a_gauche_LabyMur(); //De manière générale, il y a toujours un mur à gauche du robot
  }
  if(etat2==VIRAGE) //si le robot tourne à gauche, on compte le nombre de cases qu'il parcoure pour connaitre l'angle de rotation et la distance parcourue
  {
    if((Objet_dessous2==CASE_BLANC)&&(Objet_dessous2!=Objet_dessous1))
    {
      Labyrinthe[robotX][robotY]=BLANC;
      Tourne_Gauche_Labyrinthe();
      PasMur_en_face_LabyMur();
      Avance_Labyrinthe();
    }
    if((Objet_dessous2==CASE_ROUGE)&&(Objet_dessous2!=Objet_dessous1))
    {
      Labyrinthe[robotX][robotY]=ROUGE;
      Tourne_Gauche_Labyrinthe();
      PasMur_en_face_LabyMur();
      Avance_Labyrinthe();
    }
    if((Objet_dessous2==CASE_NOIR)&&(Objet_dessous2!=Objet_dessous1))
    {
      Labyrinthe[robotX][robotY]=NOIR;
      Tourne_Gauche_Labyrinthe();
      PasMur_en_face_LabyMur();
      Avance_Labyrinthe();
    }
  }

  if((Objet_dessous2==CASE_ROUGE)&&(Objet_dessous2!=Objet_dessous1)) //Au bout d'un tour, il affiche ce qu'il a vu
  {
    Affiche_Labyrinthe();
    Affiche_LabyMur();
  }
}

//Fonctions seulement pour afficher les changements de valeurs des variables, non utile au fonctionnement du robot
void PrintQueQuandCaChangeSens()
{
  Val1=Val2;
  Val2=Sens;
  if(Val1!=Val2)
  {
    if(Val2==HAUT) { Serial.println("HAUT");}
    if(Val2==GAUCHE) { Serial.println("GAUCHE");}
    if(Val2==BAS) { Serial.println("BAS");}
    if(Val2==DROITE) { Serial.println("DROITE");}
  }
}
void PrintQueQuandCaChangeCoordonnees()
{
  Val3=Val4;
  Val4=robotX;
  Val5=Val6;
  Val6=robotY;
  if((Val3!=Val4)||(Val5!=Val6))
  {
    Serial.print("Case : ");
    Serial.print(robotX);
    Serial.print(" ");
    Serial.println(robotY);
    robotXMur = 2*(robotX+1)-1;
    robotYMur = 2*(robotY+1)-1;
    Serial.print("Mur  : ");
    Serial.print(robotXMur);
    Serial.print(" ");
    Serial.println(robotYMur);
  }
}

//Les initialisations permettent de remettre les paramètres à leur valeurs de départ
void Initialisation() //Initailisation du programme
{
  Objet_dessous1 = INIT_Objet_dessous;
  Objet_dessous2 = INIT_Objet_dessous;
  robotX = INIT_robotX;
  robotY = INIT_robotY;
  Sens = INIT_Sens;
  robotXMur = 2*(robotX+1)-1;
  robotYMur = 2*(robotY+1)-1;
  Initialyse_Labyrinthe();
  Initialyse_LabyMur();
}
void Initialisation2() //Initialisation d'un tour de labyrinthe
{
  robotX = INIT_robotX;
  robotY = INIT_robotY-1;
  Sens = GAUCHE;
  robotXMur = 2*(robotX+1)-1;
  robotYMur = 2*(robotY+1)-1;
  Initialyse_Labyrinthe();
  Initialyse_LabyMur();
}

//Fonctions pour le labyrinthe
void Affiche_Labyrinthe()
{
  for(cursAffX=0;cursAffX<NligneLaby;cursAffX++)
  {
    for(cursAffY=0 ; cursAffY<NcolonneLaby ; cursAffY++)
    {
      Serial.print(Labyrinthe[cursAffX][cursAffY]);
      Serial.print(" ");
    }
    Serial.println("");
  }
}
void Initialyse_Labyrinthe()
{
  for(cursAffX=0;cursAffX<NligneLaby;cursAffX++)
  {
    for(cursAffY=0 ; cursAffY<NcolonneLaby ; cursAffY++)
    {
      Labyrinthe[cursAffX][cursAffY] = INCONNUE;
    }
  }
}
void Avance_Labyrinthe()
{
  //Pour savoir dans quelle direction le robot avance, il faut prendre en compte son "sens"
  if(Sens==HAUT)
  {
    robotX=robotX-1;
  }
  if(Sens==BAS)
  {
    robotX=robotX+1;
  }
  if(Sens==GAUCHE)
  {
    robotY=robotY-1;
  }
  if(Sens==DROITE)
  {
    robotY=robotY+1;
  }
}
void Tourne_Gauche_Labyrinthe()
{
  //Pour modifier le sens qui est un "enum", il faut le repaser en chiffre
  switch(Sens)
  {
    case HAUT:
      SensEnChiffre=0;
      break;
    case DROITE:
      SensEnChiffre=1;
      break;
    case BAS:
      SensEnChiffre=2;
      break;
    case GAUCHE:
      SensEnChiffre=3;
      break;
  }
  SensEnChiffre=(4+SensEnChiffre-1)%4;
  switch(SensEnChiffre)
  {
    case 0:
      Sens=HAUT;
      break;
    case 1:
      Sens=DROITE;
      break;
    case 2:
      Sens=BAS;
      break;
    case 3:
      Sens=GAUCHE;
      break;
  }
}
void Tourne_Droite_Labyrinthe()
{
  switch(Sens)
  {
    case HAUT:
      SensEnChiffre=0;
      break;
    case DROITE:
      SensEnChiffre=1;
      break;
    case BAS:
      SensEnChiffre=2;
      break;
    case GAUCHE:
      SensEnChiffre=3;
      break;
  }
  SensEnChiffre=(SensEnChiffre+1)%4;
  switch(SensEnChiffre)
  {
    case 0:
      Sens=HAUT;
      break;
    case 1:
      Sens=DROITE;
      break;
    case 2:
      Sens=BAS;
      break;
    case 3:
      Sens=GAUCHE;
      break;
  }
}

//Fonctions pour le labyMur
void Affiche_LabyMur()
{
  for(cursAffX=0;cursAffX<NligneLabyMur;cursAffX++)
  {
    for(cursAffY=0 ; cursAffY<NcolonneLabyMur ; cursAffY++)
    { 
      if((cursAffX%2==0)&&(cursAffY%2==0))
      {
        Serial.print(" ");
      }
      else
      {
        Serial.print(LabyMur[cursAffX][cursAffY]);
      }
      Serial.print(" ");
    }
    Serial.println("");
  }
}
void Initialyse_LabyMur()
{
  for(cursAffX=0;cursAffX<NligneLabyMur;cursAffX++)
  {
    for(cursAffY=0 ; cursAffY<NcolonneLabyMur ; cursAffY++)
    {
      LabyMur[cursAffX][cursAffY] = INCONNU;
    }
  }
}
void Mur_a_gauche_LabyMur()
{
  robotXMur = 2*(robotX+1)-1;
  robotYMur = 2*(robotY+1)-1;
  if(Sens==HAUT)
  {
    if(LabyMur[robotXMur][robotYMur-1]!=2)
    {
      LabyMur[robotXMur][robotYMur-1] = MUR;
    }
  }
  if(Sens==BAS)
  {
    if(LabyMur[robotXMur][robotYMur+1]!=2)
    {
      LabyMur[robotXMur][robotYMur+1] = MUR;
    }
  }
  if(Sens==GAUCHE)
  {
    if(LabyMur[robotXMur+1][robotYMur]!=2)
    {
      LabyMur[robotXMur+1][robotYMur] = MUR;
    }
  }
  if(Sens==DROITE)
  {
    if(LabyMur[robotXMur-1][robotYMur]!=2)
    {
      LabyMur[robotXMur-1][robotYMur] = MUR;
    }
  }
}
void PasMur_en_face_LabyMur()
{
  robotXMur = 2*(robotX+1)-1;
  robotYMur = 2*(robotY+1)-1;
  if(Sens==HAUT)
  {
    LabyMur[robotXMur-1][robotYMur] = PAS_MUR;
  }
  if(Sens==BAS)
  {
    LabyMur[robotXMur+1][robotYMur] = PAS_MUR;
  }
  if(Sens==GAUCHE)
  {
    LabyMur[robotXMur][robotYMur-1] = PAS_MUR;
  }
  if(Sens==DROITE)
  {
    LabyMur[robotXMur][robotYMur+1] = PAS_MUR;
  }
}

//Cette fonction permet au robot de longer le mur à sa gauche
//Il utilise les valeurs des capteur (positionés à sa gauche et devant) pour se repérer et appele les fonctions de mouvements (voir ligne 904)
//Elle renvoie également une variable "etat2" qui permet d'indiquer les actions que fait le robot (voir lignes 19 à 35 pour les variables)
void Longe_mur_gauche()
{
  distfaceG=analogRead(PROX_SENSOR_FL_PIN);
  distDgauche=analogRead(PROX_SENSOR_DL_PIN);
  distgauche=analogRead(PROX_SENSOR_L_PIN);
  etat1=etat2;
  if(distfaceG<seuil3) //Mur en face
  {
    TDroite();
    etat2=COIN;
  }
  //Les deux prochains "else if" traitent le cas où il n'y a pas de mure à gauche
  else if((distDgauche==1023)&&(distgauche<seuil1)) //si il n'y a plus de mure dans sa diagonale gauche mais qu'il le capte encore à sa gauche
  {
    MAvant(); //il avance tout droit sans réagir, il ne tournera que lorsque le mure n'est plus à sa droite, qui est le cas suivant
    etat2=LIGNE;
    boolVIRAGE=true;
  }
  else if((distDgauche>seuil2)&&(boolVIRAGE==true)) //tant qu'il n'est pas à une distance résonnable du mur, il continue de tourner
  {
    MGauche();
    etat2=VIRAGE;
  }
  //Il y a une zone entre les distances de 500 et 700 ou le robot va tout droit,
  //Si il sort de cette zone, il corrige sa trajectoire
  else if(distDgauche>seuil2) //Mur trop loin
  {
    MGauche();
  }
  else if(distDgauche<seuil1) //Mur trop pret
  {
    MDroite();
    etat2=LIGNE;
  }
  else
  {
    MAvant();
    etat2=LIGNE;
    boolVIRAGE=false;
  }
  //Afficher la réponse que lorsqu'elle a une nouvelle valeure
  /*
  if(etat1!=etat2)
  {
    if(etat2==LIGNE)
    {
      Serial.println("LIGNE ///////////////////////////////////");
    }
    if(etat2==COIN)
    {
      Serial.println("COIN ////////////////////////////////////");
    }
    if(etat2==VIRAGE)
    {
      Serial.println("VIRAGE //////////////////////////////////");
    }
  }
  */
  //Le point fort de cette fonction est qu'elle est faite de tel sorte que 
  //le robot travers toujours les lignes perpendiculairement, même dans les virages.
}
void Longe_mur_droite()
{
  distfaceD=analogRead(PROX_SENSOR_FR_PIN);
  distDdroite=analogRead(PROX_SENSOR_DR_PIN);
  distdroite=analogRead(PROX_SENSOR_R_PIN);
  etat1=etat2;
  if(distfaceD<seuil3) //Mur en face
  {
    TGauche();
    etat2=COIN;
  }
  //Les deux prochains "else if" traitent le cas où il n'y a pas de mure à droite
  else if((distDdroite==1023)&&(distdroite<seuil1)) //si il n'y a plus de mure dans sa diagonale droite mais qu'il le capte encore à sa droite
  {
    MAvant(); //il avance tout droit sans réagir, il ne tournera que lorsque le mure n'est plus à sa gauche, qui est le cas suivant
    etat2=LIGNE;
    boolVIRAGE=true;
  }
  else if((distDdroite>seuil2)&&(boolVIRAGE==true)) //tant qu'il n'est pas à une distance résonnable du mur, il continue de tourner
  {
    MDroite();
    etat2=VIRAGE;
  }
  //Il y a une zone entre les distances de 500 et 700 ou le robot va tout droit,
  //Si il sort de cette zone, il corrige sa trajectoire
  else if(distDdroite>seuil2) //Mur trop loin
  {
    MDroite();
  }
  else if(distDdroite<seuil1) //Mur trop pret
  {
    MGauche();
    etat2=LIGNE;
  }
  else
  {
    MAvant();
    etat2=LIGNE;
    boolVIRAGE=false;
  }
  //Afficher la réponse que lorsqu'elle a une nouvelle valeure
  /*
  if(etat1!=etat2)
  {
    if(etat2==LIGNE)
    {
      Serial.println("LIGNE ///////////////////////////////////");
    }
    if(etat2==COIN)
    {
      Serial.println("COIN ////////////////////////////////////");
    }
    if(etat2==VIRAGE)
    {
      Serial.println("VIRAGE //////////////////////////////////");
    }
  }
  */
}

//Cette fonction permet de savoir si le robot est sur une case ou une ligne, et de quelle couleur elle est.
//Cette fonction utilise la réponse de la fonction "Estime_couleur" et utilise la fonction "millis()" pour savoir si c'est une ligne ou une case
//Elle donne enfin un nom à la variable "Objet_dessous" (voir lignes 47 à 57 pour les variables).
void Voir_Quadrillage()
{
  Estime_Couleur();
  Objet_dessous1=Objet_dessous2;
  if((couleur_case1!=couleur_case2)&&(couleur_case2!=3))
  //Cette ligne permet de réfléchir que si la couleur est connue
  {
    //En suite, on associe les couleurs estimées à des "cases" au sol.
    couleur_caseAvant=couleur_caseApres;
    temps_couleur1=temps_couleur2;
    couleur_caseApres=couleur_case2;
    temps_couleur2=millis();
    temps_couleur_case=temps_couleur2-temps_couleur1;
    //si la "case" a été traversée vite, c'est une ligne, mais il faut faire attention aux combinaisons impossibles
    if(couleur_caseAvant==ROUGE)
    {
      if(!(temps_couleur_case<seuil_temps_case_ligne))
      {
        Objet_dessous2=CASE_ROUGE;
      }
    } 
    if(couleur_caseAvant==NOIR)
    {
      if(temps_couleur_case<seuil_temps_case_ligne)
      {
        Objet_dessous2=LIGNE_NOIR;
      }
      else
      {
        Objet_dessous2=CASE_NOIR;
      }
    } 
    if(couleur_caseAvant==BLANC)
    {
      Objet_dessous2=CASE_BLANC;
    }
  }
 //Afficher la réponse que lorsqu'elle a une nouvelle valeure
  /*
  if(Objet_dessous1!=Objet_dessous2)
  {
    if(Objet_dessous2==LIGNE_NOIR)
    {
      Serial.println("LIGNE_NOIR");
    }
    if(Objet_dessous2==CASE_NOIR)
    {
      Serial.println("CASE_NOIR");
    }
    if(Objet_dessous2==CASE_BLANC)
    {
      Serial.println("CASE_BLANC");
    }
    if(Objet_dessous2==CASE_ROUGE)
    {
      Serial.println("CASE_ROUGE");
    }
  }
  */
  //Bien sure, cette fonction est infaillible seulemet si la trajectoire du robot 
  //est toujours perpendiculaire aux lignes qu'il traverse. Ce qui est toujours le cas.
}

//Cette fonction permet d'identifier la couleur qui est sous le robot grâce au capteur
// de couleur. Cette fonction donne un etat à la variable "couleur_case2" (voir lignes 37 à 46 pour les variables)
// Cette fonction utilise des seuils pour déterminer la couleur.
void Estime_Couleur()
{
  //Pour mesurer la couleur, il faut être sure qu'il s'agit de la bonne mesure
 //c'est pourquoi l'analyse de la réponse du capteur se fait sur les deux dernières mesures prises
 mesure_couleur1=mesure_couleur2;
 couleur_case1=couleur_case2;
 mesure_couleur2=analogRead(LIGHT_SENSOR_PIN);
 //En fonction de la valeur des deux dernières mesures, la fonction attribue sa valeur à "couleur-case2"
 if((mesure_couleur1>seuilBLANC)&&(mesure_couleur2>seuilBLANC))
 {
  couleur_case2=BLANC;
 }
 else if(((mesure_couleur1>seuilROUGEbas)&&(mesure_couleur2>seuilROUGEbas))&&((mesure_couleur1<seuilROUGEhaut)&&(mesure_couleur2<seuilROUGEhaut)))
 {
  couleur_case2=ROUGE;
 }
 else if((mesure_couleur1<seuilNOIR)&&(mesure_couleur2<seuilNOIR))
 {
  couleur_case2=NOIR;
 }
 else
 {
  couleur_case2=INCONNUE;
 }
}

//Les 7 fonctions suivantes commandent tout les mouvements possibles que peut faire le robot. 
//Ils comandent les moteurs des roues.
void MGauche() //Avancer en tournant à gauche
{
  if(Depart==true)
  {
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  analogWrite(MOTOR_R_SPEED, vit);
  analogWrite(MOTOR_L_SPEED, vit/1.7);
  //Serial.println("MGauche");
  //MStop();
  }
}
void MDroite() //Avancer en tournant à droite
{
  if(Depart==true)
  {
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  analogWrite(MOTOR_R_SPEED, vit/1.7);
  analogWrite(MOTOR_L_SPEED, vit);
  //Serial.println("MDroite");
  //MStop();
  }
}
void TDroite() //Tourner à droite sur lui même
{
  if(Depart==true)
  {
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  analogWrite(MOTOR_R_SPEED, vit);
  analogWrite(MOTOR_L_SPEED, vit);
  //Serial.println("TDroite");
  //MStop();
  }
}
void TGauche() //Tourner à gauche sur lui-même
{
  if(Depart==true)
  {
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);
  analogWrite(MOTOR_R_SPEED, vit);
  analogWrite(MOTOR_L_SPEED, vit);
  //Serial.println("TGauche");
  //MStop();
  }
}
void MAvant() //Avancer tout droit
{
  if(Depart==true)
  {
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  analogWrite(MOTOR_R_SPEED, vit);
  analogWrite(MOTOR_L_SPEED, vit);
  //Serial.println("MAvant");
  //MStop();
  }
}
void MArriere() //Reculer tout droit
{
  if(Depart==true)
  {
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);
  analogWrite(MOTOR_R_SPEED, vit);
  analogWrite(MOTOR_L_SPEED, vit);
  //Serial.println("MArriere");
  //MStop();
  }
}
void MStop() //S'arrêter
{
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, LOW);
  //Serial.println("MStop");
}

//Ces fonctions là font la même chose mais avec une vitesse différente
void Avancer_Aleatoire()
{
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  analogWrite(MOTOR_R_SPEED, vitesse_max_A);
  analogWrite(MOTOR_L_SPEED, vitesse_max_A);

}
void Tourner_Droite_Aleatoire()
{
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  analogWrite(MOTOR_R_SPEED, vitesse_min_A);
  analogWrite(MOTOR_L_SPEED, vitesse_max_A);
}
void Tourner_Gauche_Aleatoire()
{
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);
  analogWrite(MOTOR_R_SPEED, vitesse_max_A);
  analogWrite(MOTOR_L_SPEED, vitesse_min_A);
}
