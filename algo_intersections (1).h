/* BA Systemes
 * 
 * Projet: 0000Z - SPEED 
 * Composants: C_CIRCUIT_LASER-01
 * Author : Rajkrishna Paul 
 * __IDCVS__
 */

// ----------------------------------------------------------------------------

#ifndef __ALGO_INTERSECTIONS_H__
#define __ALGO_INTERSECTIONS_H__

// ------ DI - directives d'inclusion -----------------------------------------

#include "track/lib/algo_cmp.h"
#include "track/lib/algo_cmp_zone.h"
#include "tools/ba/misc/util.h"
#include "track/lib/point_tube.h"

#include <vector>

using namespace std;


// ------ CP - constantes publiques -------------------------------------------

// ------ MP - macros publiques -----------------------------------------------

// ------ TP - types et structures publics ------------------------------------

// ------ DC - déclaration des classes ----------------------------------------


/* Algo permettant de calculer les points du polygone d'un AGV, d'une porte...
 * l'algo met à jour un vecteur de 4 pointGeom_t : Ar Gauche, Av Gauche, Av Droit, Ar Droit
*/
class AlgoCalculPolygone : public AlgoCmp
{
    ISA(AlgoCalculPolygone, AlgoCmp);
public:
     

    AlgoCalculPolygone( vector<pointGeom_t>* polygone,
                        double x, double y, double gamma,
                        double xAv, double xAr,
                        double yGauche, double yDroit,
                        CmpDataManager* cmpDataManager) :
        AlgoCmp(cmpDataManager),
        m_polygone( polygone ),
        m_x(x),
        m_y(y),
        m_gamma(gamma),
        m_xAv(xAv),
        m_xAr(xAr),
        m_yGauche(yGauche),
        m_yDroit(yDroit)
        {}

    void setValues( vector<pointGeom_t>* polygone,
                    double x, double y, double gamma,
                    double xAv, double xAr,
                    double yGauche, double yDroit );

    status_t run();
    virtual ~AlgoCalculPolygone() {}
private:
    // polygone
    vector<pointGeom_t>* m_polygone; ///< polygone representant l'AGV, la porte...
    double m_x; ///< position x du centre
    double m_y; ///< position y du centre
    double m_gamma; ///< angle
    double m_xAv; ///< distance en x de l'avant par rapport au centre (positive)
    double m_xAr; ///< distance en x de l'arriere par rapport au centre (negative)
    double m_yGauche; ///< distance en y de la gauche par rapport au centre (positive)
    double m_yDroit; ///< distance en y de la droite par rapport au centre (negative)
};


/* algo permettant de calculer les points d'intersections entre deux polygones
 * l'algo met à jour un vecteur de PointCircuit
*/
class AlgoIntersectionPairePolygone : public AlgoCmp
{
  ISA(AlgoIntersectionPairePolygone, AlgoCmp);
public:
   

  AlgoIntersectionPairePolygone(const vector<pointGeom_t> *poly1, const vector<pointGeom_t> *poly2,
          vector<pointGeom_t> *res, bool finirVite, CmpDataManager *cmpDataManager) :
    AlgoCmp(cmpDataManager),
    m_poly1( poly1 ),
    m_poly2( poly2 ),
    m_res( res ),
    m_finirVite(finirVite)
  {}

  void setValues( const vector<pointGeom_t> *poly1,
                  const vector<pointGeom_t> *poly2,
                  vector<pointGeom_t> *res,
                  bool finirVite );

  status_t run();
  virtual ~AlgoIntersectionPairePolygone() {}
private:
  // polygone 1
  const vector<pointGeom_t> *m_poly1;
  // polygone 2
  const vector<pointGeom_t> *m_poly2;
  // resultat du calcul sous forme de point circuit
  vector<pointGeom_t> *m_res;
  // sortir du calcul à la première intersection trouvée ?
  bool m_finirVite;
};


/** Recherche des objets se trouvant dans un espace definit par une liste de pointGeom_t.
 * La liste de pointGeom_t est fournie dans le constructeur.
 */
class AlgoPointIsInSpace : public AlgoCmp
{
  ISA(AlgoPointIsInSpace, AlgoCmp);
public:
   
  /** algorithme de recherche de presence dans un espace
   * @param abcisse du point
   * @param ordonnee du point
   * @parma *isInSpace indicateur de presence
   * @param cmpDataManager gestionnaire de composants.
   */
  AlgoPointIsInSpace( double x, double y,
                      const vector<pointGeom_t>* lstPointGeom,
                      bool *isInSpace,
                      CmpDataManager *cmpDataManager ) :
    AlgoCmp(cmpDataManager),
    m_x(x), m_y(y), m_lstPointGeom(lstPointGeom), m_isInSpace(isInSpace)
  {}

  status_t run();
  virtual ~AlgoPointIsInSpace() {}
  void setValues(double x, double y, const vector<pointGeom_t>* lstPointGeom, bool* isInSpace);
private:
  /// abscisse du point a etudier
  double m_x;
  /// ordonnee du point a etudier
  double m_y;
  /// liste de points geometriques definissant l'espace
  const vector<pointGeom_t> *m_lstPointGeom;
  /// booleen de retour : true si le point est dans l'espace, false sinon
  bool *m_isInSpace;
};

/* algo permettant de calculer les points d'intersections entre deux AGV
 *
*/
class AlgoIntersectionAgv : public AlgoCmp
{
  ISA(AlgoIntersectionAgv, AlgoCmp);
public:
   

  AlgoIntersectionAgv(double x1, double y1, double theta1, ident_t idAgv1, bool isTheoric1,
                      double x2, double y2, double theta2, ident_t idAgv2, bool isTheoric2,
                      bool *choc, CmpDataManager *dm) :
    AlgoCmp(dm),
    m_x1(x1),
    m_y1(y1),
    m_theta1(theta1),
    m_idAgv1(idAgv1),
    m_isTheoric1(isTheoric1),
    m_x2(x2),
    m_y2(y2),
    m_theta2(theta2),
    m_idAgv2(idAgv2),
    m_isTheoric2(isTheoric2),
    m_choc(choc)
  {}

  status_t run();
  void setValues(double x1, double y1, double theta1, ident_t idAgv1, bool isTheoric1,
                 double x2, double y2, double theta2, ident_t idAgv2, bool isTheoric2,
                 bool *choc);

  virtual ~AlgoIntersectionAgv() {}
private:
  double m_x1;
  double m_y1;
  double m_theta1;
  ident_t m_idAgv1;
  bool m_isTheoric1;
  double m_x2;
  double m_y2;
  double m_theta2;
  ident_t m_idAgv2;
  bool m_isTheoric2;
  bool *m_choc;
};
/* algo permettant de calculer les points d'intersections entre deux AGV
 *
*/
class AlgoIntersectionAgvSansDiscrimination : public AlgoCmp
{
  ISA(AlgoIntersectionAgvSansDiscrimination, AlgoCmp);
public:
   

  AlgoIntersectionAgvSansDiscrimination(double x1, double y1, double theta1, ident_t idAgv1, bool isTheoric1,
                      double x2, double y2, double theta2, ident_t idAgv2, bool isTheoric2,
                      bool *choc, CmpDataManager *dm) :
    AlgoCmp(dm),
    m_x1(x1),
    m_y1(y1),
    m_theta1(theta1),
    m_idAgv1(idAgv1),
    m_isTheoric1(isTheoric1),
    m_x2(x2),
    m_y2(y2),
    m_theta2(theta2),
    m_idAgv2(idAgv2),
    m_isTheoric2(isTheoric2),
    m_choc(choc)
  {}

  status_t run();
  virtual ~AlgoIntersectionAgvSansDiscrimination() {}
  void setValues( double x1, double y1, double theta1, ident_t idAgv1, bool isTheoric1,
                  double x2, double y2, double theta2, ident_t idAgv2, bool isTheoric2,
                  bool *choc);

private:
  double m_x1;
  double m_y1;
  double m_theta1;
  ident_t m_idAgv1;
  bool m_isTheoric1;
  double m_x2;
  double m_y2;
  double m_theta2;
  ident_t m_idAgv2;
  bool m_isTheoric2;
  bool *m_choc;
};


/* algo permettant de calculer une liste de points correspondant à un troncon
 *
*/
class AlgoListPointTrc : public AlgoCmp
{
  ISA(AlgoListPointTrc, AlgoCmp);
public:
   

  AlgoListPointTrc(ident_t idTrc, ident_t idAgv, listPointTube_t *listPoint, CmpDataManager *dm) :
    AlgoCmp(dm),
    m_idTrc(idTrc),
    m_idAgv(idAgv),
    m_listPoint(listPoint),
    m_s(0),
    m_longueurOffset(0)
  {}

  status_t run();
  virtual ~AlgoListPointTrc() {}
  void setValues(ident_t idTrc, bool marcheAv, ident_t idAgv, listPointTube_t *listPoint,
    double s = 0, double longueurOffset = 0);
private:
  ident_t m_idTrc;
  bool m_marcheAv;
  ident_t m_idAgv;
  listPointTube_t *m_listPoint;
  double m_s;
  double m_longueurOffset;
};

/**
 * algo permettant de calculer les tronçons sous un point
 */
class AlgoIntersectionTrc : public AlgoCmp
{
  ISA(AlgoIntersectionTrc, AlgoCmp);
public:
   

  AlgoIntersectionTrc(DataAgv* dataAgv, double x, double y, double radius, ListIdent &listTrc, CmpDataManager *dm) :
    AlgoCmp(dm),
    m_x(x),
    m_y(y),
	m_radius(radius),
    m_listTrc(listTrc),
	m_dataAgv(dataAgv)
  {}
  virtual ~AlgoIntersectionTrc() {}

  status_t run();
  void setValues(DataAgv* dataAgv, double x, double y, double radius, ListIdent &listTrc)
  {
	    m_x = x;
	    m_y = y;
		m_radius = radius;
	    m_listTrc = listTrc;
	    m_dataAgv = dataAgv;
  }

private:
  double m_x;
  double m_y;
  double m_radius;
  ListIdent &m_listTrc;
  DataAgv* m_dataAgv;
};


/* algo permettant de calculer les tronçons gênés par la présence d'un AGV
 *
*/
class AlgoIntersectionAgvTrc : public AlgoCmp
{
  ISA(AlgoIntersectionAgvTrc, AlgoCmp);
public:
   

  AlgoIntersectionAgvTrc(double x, double y, double theta, ident_t idAgv, ident_t idAgvCible,
                         ListIdent &listTrc, CmpDataManager *dm) :
    AlgoCmp(dm),
    m_x(x),
    m_y(y),
    m_theta(theta),
    m_idAgv(idAgv),
    m_idAgvCible(idAgvCible),
    m_listTrc(listTrc)
  {}

  status_t run();
  void setValues(double x, double y, double theta, ident_t idAgv, ident_t idAgvCible, ListIdent &listTrc)
  {
	    m_x = x;
	    m_y = y;
	    m_theta = theta ;
	    m_idAgv = idAgv;
	    m_idAgvCible = idAgvCible;
	    m_listTrc = listTrc;
  }

  virtual ~AlgoIntersectionAgvTrc() {}
private:
  double m_x;
  double m_y;
  double m_theta;
  ident_t m_idAgv;
  ident_t m_idAgvCible;
  ListIdent &m_listTrc;
};


/** algo permettant de calculer l'intersection entre 2 points cibles
 * @param ptProprio position de l'AGV propriétaire
 * @param rayonProprio rayon de l'AGV propriétaire
 * @param ptCible position de l'AGV cible
 * @param rayonCible rayon de l'AGV cible
*/
class AlgoIsIntersectionPointTube : public AlgoCmp
{
  ISA(AlgoIsIntersectionPointTube, AlgoCmp);
public:
   

  AlgoIsIntersectionPointTube( PointTube *ptProprio,
                               PointTube *ptCible,
                               bool *isIntersection,
                               double* distCarre,
                               CmpDataManager *dm ):
      AlgoCmp(dm),
      m_ptProprio(ptProprio),
      m_ptCible(ptCible),
      m_isIntersection(isIntersection),
      m_distanceCarre(distCarre)
      {}

  status_t run();
  void setValues(PointTube *ptProprio,
                 PointTube *ptCible,
                 bool *isIntersection,
                 double* distCaree);

  virtual ~AlgoIsIntersectionPointTube() {}
private:
  PointTube* m_ptProprio; ///< position de l'AGV propriétaire
  PointTube* m_ptCible; ///< position de l'AGV cible
  bool *m_isIntersection; ///valeur de retour : il y a intersection ou non
  double* m_distanceCarre; ///< distance au carre entre les deux points tube (carre pour eviter des sqrt)
};


/** algo permettant de recalculer la partie vitesse d'un point tube
 * @param vitesse : vitesse de l'AGV (>0 -> marche avant, <0 -> marche arriere)
 * @param vitesseCurrentComputation : vitesse de l'AGV pour le calcul courant
 * @param checkZoneRalenti : Doit on tester si le point est dans une zone de ralenti ?
 *        en pratique: si la vitesse est remontée par l'AGV -> false,
 *                     si la vitesse est celle du troncon -> true
 * @param pointTube dont on informe les coordonnées et remplit le polygone dans l'algo
 * @param champSecuMini : champ secu minimum
*/
class AlgoChangeVitessePointTube : public AlgoCmp
{
  ISA(AlgoChangeVitessePointTube, AlgoCmp);
public:
   

  /// Constructeur (parametres : voir au dessus)
  AlgoChangeVitessePointTube(DataAgv* dataAgv,
                        int16_t vitesse,
                        int16_t vitesseCurrentComputation,
                        bool checkZoneRalenti,
                        PointTube *pointTube,
                        const GeomChampSecurite& champSecuMini,
                        CmpDataManager* dm)
    : AlgoCmp(dm),
      m_dataAgv(dataAgv),
      m_vitesse(vitesse),
      m_vitesseCurrentComputation(vitesseCurrentComputation),
      m_checkZoneRalenti(checkZoneRalenti),
      m_pointTube(pointTube),
      m_champSecuMini(champSecuMini)
    {}

  status_t run();

  /// setValues (parametres : voir au dessus)
  void setValues( DataAgv* dataAgv,
                  int16_t vitesse,
                  int16_t vitesseCurrentComputation,
                  bool checkZoneRalenti,
                  PointTube *pointTube,
                  const GeomChampSecurite& champSecuMini);

  virtual ~AlgoChangeVitessePointTube() {}

  PointTube* getPointTube() const { return m_pointTube; }
  DataAgv* getDataAgv() const { return m_dataAgv; }

private:
  DataAgv* m_dataAgv; ///< agv avec lequel on va remplir notre point tube
  int16_t m_vitesse; ///< vitesse de l'AGV sur ce point
  int16_t m_vitesseCurrentComputation; ///< vitesse de l'AGV sur ce point pour le calcul en cours
  bool m_checkZoneRalenti; ///< Doit on tester si le point est dans une zone de ralenti ?
  PointTube* m_pointTube; ///< point tube qui sera modifié
  GeomChampSecurite m_champSecuMini; ///< champ de securite mini a appliquer (si passe en param, la variable m_computeChampSecuMini est inutile)
};


/** algo permettant de calculer un point tube
 * @param x abscisse du point tube
 * @param y ordonnee du point tube
 * @param theta du point tube
 * @param marcheAv Marche avant/arriere
 * @param vitesse : vitesse de l'AGV (>0 -> marche avant, <0 -> marche arriere)
 * @param checkZoneRalenti : Doit on tester si le point est dans une zone de ralenti ?
 *        en pratique: si la vitesse est remontée par l'AGV -> false,
 *                     si la vitesse est celle du troncon -> true
 * @param isPointTubeTheoric : le point tube est-il theorique (difference de marges)
 * @param pointTube dont on informe les coordonnées et remplit le polygone dans l'algo
 * @param champSecuMini : champ secu minimum
*/
class AlgoCalculPointTube : public AlgoCmp
{
  ISA(AlgoCalculPointTube, AlgoCmp);
public:
   

  /// Constructeur (parametres : voir au dessus)
  AlgoCalculPointTube(DataAgv* dataAgv,
                      double x,
                      double y,
                      double theta,
                      bool marcheAv,
                      int16_t vitesse,
                      bool checkZoneRalenti,
                      bool isPointTubeTheoric,
                      PointTube *pointTube,
                      const GeomChampSecurite& champSecuMini,
                      CmpDataManager* dm)
    : AlgoCmp(dm),
      m_algoChangeVitessePointTube(dataAgv,
                              vitesse,
                              0,
                              checkZoneRalenti,
                              pointTube,
                              champSecuMini,
                              dm),
      m_x(x),
      m_y(y),
      m_theta(theta),
      m_marcheAv(marcheAv),
      m_isPointTubeTheoric(isPointTubeTheoric)
    {}

  status_t run();

  /// setValues (parametres : voir au dessus)
  void setValues( DataAgv* dataAgv,
                  double x,
                  double y,
                  double theta,
                  bool marcheAv,
                  int16_t vitesse,
                  bool checkZoneRalenti,
                  bool isPointTubeTheoric,
                  PointTube *pointTube,
                  const GeomChampSecurite& champSecuMini);

  virtual ~AlgoCalculPointTube() {}

private:
  AlgoChangeVitessePointTube m_algoChangeVitessePointTube;
  double m_x; ///< abscisse du point tube
  double m_y; ///< ordonne du point tube
  double m_theta; ///< theta du point tube
  bool m_marcheAv; ///< marche avant/arriere
  bool m_isPointTubeTheoric; ///< indique si le point tube est theorique ou non (difference de marge)
};


/** Algo de recherche d'interaction entre un AGV sur un point tub et un troncon
 * @param x abscisse du point tube
 * @param y ordonnee du point tube
 * @param theta du point tube
 * @param vitesse : vitesse de l'AGV (>0 -> marche avant, <0 -> marche arriere)
 * @param checkZoneRalenti : Doit on tester si le point est dans une zone de ralenti ?
 *        en pratique: si la vitesse est remontée par l'AGV -> false,
 *                     si la vitesse est celle du troncon -> true
 * @param pointTube dont on informe les coordonnées et remplit le polygone dans l'algo
*/
class AlgoIsInteractionAgvTrc : public AlgoCmp
{
  ISA(AlgoIsInteractionAgvTrc, AlgoCmp);
public:
   

  AlgoIsInteractionAgvTrc(ident_t idTrc,
                          PointTube* pointTube,
                          DataAgv* dataAgv,
                          bool checkZoneRalenti,
                          bool *isInteraction,
                          CmpDataManager* dm)
  : AlgoCmp(dm),
    m_idTrc(idTrc),
    m_pointTube(pointTube),
    m_dataAgv(dataAgv),
    m_checkZoneRalenti(checkZoneRalenti),
    m_isInteraction(isInteraction)
   {}

  status_t run();
  void setValues( ident_t idTrc,
                  PointTube *pointTube,
                  DataAgv* dataAgv,
                  bool checkZoneRalenti,
                  bool *isInteraction);

  virtual ~AlgoIsInteractionAgvTrc() {}

private:
  ident_t m_idTrc; ///< identifiant de troncon
  PointTube* m_pointTube; ///<point tube
  DataAgv* m_dataAgv; ///< agv avec lequel on va remplir notre point tube
  bool m_checkZoneRalenti; ///< Doit on tester si le point est dans une zone de ralenti ?
  bool *m_isInteraction; ///< booleen indiquant s'il y a interaction
};



//---------------------------------------------------------------------------
#endif
