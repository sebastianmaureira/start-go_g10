// quel module de suivi de ligne faut-il utiliser ?
// Dans les deux cas, il faudra proposer un code qui met à jour une variable globale
// représentant l'état des détecteurs
// Cf. exemple Makeblock pour le module

// Module utilisé
#define USE_RGB
typedef int T_octet; 
// Connecteur utilisé 
#define LINEFOLLOWER PORT_7





// NE PAS EDITER SOUS CETTE LIGNE /////////////////////

#if defined(USE_RGB)
  // cas de l'utilisation d'un module à quatre capteurs
  #define LF_SIZE 4
#elif defined(USE_ARRAY)
	// cas de l'utilisation d'un module à 6 capteurs
	#define LF_SIZE 6
#else
  // cas de l'utilisation d'un module à 2 capteurs
	#define LF_SIZE 2
#endif

/*  Cf.  MeMegaPi.h
 *   MePort_Sig mePort[17] = ...
        5           6             7             8 
   { 16, 17 }, {  A8,  A9 }, { A10, A11 }, { A13, A12 }, 
 */
 
#if (LINEFOLLOWER==PORT_5)
  #define LFDATAPIN 17
#endif 
#if (LINEFOLLOWER==PORT_6)
  #define LFDATAPIN A9
#endif 
#if (LINEFOLLOWER==PORT_7)
  #define LFDATAPIN A11
#endif 
#if (LINEFOLLOWER==PORT_8)
  #define LFDATAPIN A12
#endif 


// résultat du capteur : 1 octet
extern T_octet lfSensor; 

// Allume <=> 1 <=> BLANC
#define BLANC 1
#define NOIR 0

#define COL(s) ((s == BLANC) ? "B" : "N")
#define COULEUR(s) ((s == BLANC) ? "Blanc" : "Noir")

int positionRealtive();
void lineFollower_setup();
int positionRelative();
unsigned int turnDirection(unsigned int var);
uint8_t bp();
