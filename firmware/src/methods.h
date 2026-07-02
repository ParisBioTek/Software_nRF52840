#ifndef METHODS_H_
#define METHODS_H_

/* Techniques d'analyse électrochimique sélectionnables depuis le PC (SMP START).
 * L'entier est transmis tel quel dans la clé "method" du payload CBOR. */
typedef enum {
    METHOD_CV  = 0,   /* voltampérométrie cyclique (RampTest.c / CV.c)          */
    METHOD_CA  = 1,   /* chronoampérométrie i–t (CA.c, logiciel-timé)           */
    METHOD_DPV = 2,   /* voltampérométrie à impulsions différentielles (DPV.c)  */
} analysis_method_t;

#endif /* METHODS_H_ */
