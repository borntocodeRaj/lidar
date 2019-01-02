/* BA Systemes
 * 
 * Projet: 0000Z - SPEED 
 * Composants: C_CIRCUIT_LASER
 * Rajkrishna Paul
 * __IDCVS__
 */

// ----------------------------------------------------------------------------

#ifndef __ALGO_NOTIFY_POINT_CONNEXION_HH__
#define __ALGO_NOTIFY_POINT_CONNEXION_HH__

// ------ DI - directives d'inclusion -----------------------------------------

#include "track/lib/algo_notify.h"

// ------ Cp - constantes publiques -------------------------------------------

// ------ MP - macros publiques -----------------------------------------------

// ------ TP - types et structures publics ------------------------------------

// ------ DC - declaration des classes ----------------------------------------

/** Algorithme de notification de modification de point de connexion
 *  La modification consiste en :
 *  . la modification des coordonnees du ptCon
 *  @param idPtCon identifiant du point de connexion qu'on modifie
 *  @param depBase true si deplacement base associee
 *  @param x nouvelle abscisse
 *  @param y nouvelle ordonnee
 *  @param gamma nouvel angle
 */
class AlgoNotifyCoordOfPtCon : public AlgoNotify
{
  ISA(AlgoNotifyCoordOfPtCon, AlgoNotify);
public:
   
  AlgoNotifyCoordOfPtCon( ident_t idPtCon,
                          bool depBase,
                          double x, double y, double gamma,
                          CmpDataManager *cmpDataManager, uint32_t noHorizontal);
  status_t doo();
  void undoo();
  virtual ~AlgoNotifyCoordOfPtCon();
private:
  ident_t m_idPtCon; 
  bool m_depBase;
  double m_x;
  double m_y;
  double m_gamma;
  DataPointConnexion *m_data;
};

/** Algorithme de notification de modification d'alaph de point de connexion
 *  La modification consiste en :
 *  . la mise à jour d'alpha
 *  . la mise à jour des graph pour les arcs limitrophes
 *  @param idPtCon identifiant du pt de connexion qu'on modifie
 *  @param alpha angle en radian (relatif)
 */
class AlgoNotifyAlphaOfPtCon : public AlgoNotify
{
  ISA(AlgoNotifyAlphaOfPtCon, AlgoNotify);
public:
   
  AlgoNotifyAlphaOfPtCon( ident_t idPtCon, double alpha,
                         CmpDataManager *cmpDataManager, uint32_t noHorizontal );
  status_t doo();
  void undoo();
  virtual ~AlgoNotifyAlphaOfPtCon();
private:
  ident_t m_idPtCon;
  double m_alpha;
  bool m_majArcEffectuee;
  ListIdent* m_listTrc;
};

/** Algorithme de notification de modification de numero (client)
 *  La modification consiste en :
 *  . la modification du numero de clainet de la pointConnexion
 *  . la modification dans la map de la liste des dataPointConnexions
 *  @param idPointConnexion identifiant de la branche qu'on modifie
 *  @param idNum nouvel ou ancien identifiant du point de notre brc
 */
class AlgoNotifyNumOfPointConnexion : public AlgoNotify
{
  ISA(AlgoNotifyNumOfPointConnexion, AlgoNotify);
public:
   
  AlgoNotifyNumOfPointConnexion( ident_t idPointConnexion,
                                uint32_t numPointConnexion,
                                CmpDataManager *cmpDataManager,
                                uint32_t noHorizontal );
  status_t doo();
  void undoo();
  virtual ~AlgoNotifyNumOfPointConnexion();
private:
  ident_t m_idPointConnexion;
  uint32_t m_numPointConnexion;

  DataPointConnexion *m_data;
};

/** Algorithme de notification de modification de numero (client)
 *  La modification consiste en :
 *  . la modification du numero de clainet de la pointConnexion
 *  . la modification dans la map de la liste des dataPointConnexions
 *  @param idPointConnexion identifiant de la branche qu'on modifie
 *  @param idNum nouvel ou ancien identifiant du point de notre brc
 */
class AlgoNotifyOrientationOfPointConnexion : public AlgoNotify
{
        ISA(AlgoNotifyOrientationOfPointConnexion, AlgoNotify);
    public:
         
        AlgoNotifyOrientationOfPointConnexion( ident_t idPointConnexion,
                bool directOrientation,
                bool indirectOrientation,
                CmpDataManager *cmpDataManager,
                uint32_t noHorizontal );
        status_t doo();
        void undoo();
        virtual ~AlgoNotifyOrientationOfPointConnexion();
        ListIdent* getListIdAccDeleted() { return m_listIdAccDeleted;};
    private:
        ident_t m_idPointConnexion;
        bool m_directOrientation;
        bool m_indirectOrientation;

        DataPointConnexion *m_data;
        ListIdent* m_listTrcModifie;
        ListIdent* m_listIdAccDeleted;
};

// ------ PF - prototypes des fonctions publiques -----------------------------

// ------ VG - variables globales exportées -----------------------------------

// ----------------------------------------------------------------------------
#endif  // __ALGO_NOTIFY_POINT_CONNEXION_HH__
// ------ FIN -----------------------------------------------------------------
