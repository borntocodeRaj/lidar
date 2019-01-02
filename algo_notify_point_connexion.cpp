/* BA Systemes
 *
 * Projet: 0000Z - SPEED
 * Composants: voir JYB
 * Rajkrishna Paul
 * __IDCVS__
 */

// ----------------------------------------------------------------------------

/** @file
 * Implementation des algorithmes de notification de modif de point de connexion
 *
 * Implementation des algorithmes de :
 * - modification de troncon ou de distance sur le troncon d un pt de connexion
 * - modification de type de point de connexion
 * - ...
 */

// ------ DI - directives d'inclusion -----------------------------------------
#include "track/lib/algo_notify_point_connexion.h"
#include "track/lib/algo_notify_troncon.h"
#include "track/lib/algo_cmp_troncon.h"
#include "track/lib/algo_data_accostage.h"
#include "track/lib/algo_cmp_base.h"
#include "track/lib/algo_notify_base.h"
#include "tools/ba/misc/util.h"

// ------ CP - constantes privées ---------------------------------------------

// ------ MP - macros privées -------------------------------------------------

// ------ TP - types, structures et classes privés ----------------------------

// ------ VG - variables globales publiques et privées ------------------------

// Initialiser toutes les variables de classes ici.

// ------ PF - prototypes des fonctions privées -------------------------------

// ------ CF - code des méthodes ----------------------------------------------

// AlgoNotifyCoordOfPtCon ------------------------------------------------------

/** Constructeur de l algo de modif de Point de connexion par troncon
 *  Fait une copie de sauvegarde du composant et de son data
 */
AlgoNotifyCoordOfPtCon::AlgoNotifyCoordOfPtCon( ident_t idPtCon,
                                                bool depBase,
                                                double x,
                                                double y,
                                                double gamma,
                                                CmpDataManager *cmpDataManager,
                                                uint32_t noHorizontal)
  : AlgoNotify(cmpDataManager, noHorizontal),
    m_idPtCon(idPtCon),
    m_depBase(depBase),
    m_x(x),
    m_y(y),
    m_gamma(gamma)
{
  m_data = getCmpDataManager()->getDataPointConnexion(m_idPtCon);
  m_dataSaved = NULL;
  if (m_data == NULL)
    {
      BA_L(LOG_ERROR,"Modification de composant impossible");
    }
  else
    {
      m_dataSaved = m_data->getCopy();
    }
};

/** Destructeur de l algo de modif de Point de connexion par troncon
 *  Detruit les copie de sauvegarde du composant et de son data
 */
AlgoNotifyCoordOfPtCon::~AlgoNotifyCoordOfPtCon()
{
  m_data = NULL;
  if (m_dataSaved != NULL)
    {
      delete m_dataSaved;
      m_dataSaved = NULL;
    }
}

/** Algorithme de notification de modification de point de connexion
 *  La modification consiste en :
 *  . la modification des coordonnees du ptCon
 */
status_t
AlgoNotifyCoordOfPtCon::doo()
{
  status_t res=STATUS_OK;
  ListIdent *listIdent;
  AlgoNotifyPtConOfTrc* algoTrc;
  ident_t idTrc;
  DataTroncon* dataTrc;
  string outMsg;
  DataPointConnexion* dataTmp;
  ListDataCircuit::mapUnicity_t::const_iterator iterBegin, iterEnd, it;
  const ListDataCircuit::mapUnicity_t* mapUnicity =
      getCmpDataManager()->getMapUnicity(DataManager::NAME_POINT_CONNEXION);
  int32_t margeBasse, margeHaute;
  CmpDataManager* dm = getCmpDataManager();

  if (m_data == NULL)
    {
      BA_L(LOG_ERROR,"Modification de composant impossible");
      return STATUS_ERROR;
    }

  m_data->setX(m_x);
  m_data->setY(m_y);
  m_data->setGamma(m_gamma);

  outMsg = "Modification de coordonnees du pt de connexion '"
            + m_data->getName() + "'";
  addInfoNotif(outMsg, STATUS_OK, m_data->getIdent());

  margeBasse = m_data->getSpecifity()-static_cast<int32_t>(ListDataCircuit::m_epsilonX);
  margeHaute = m_data->getSpecifity()+static_cast<int32_t>(ListDataCircuit::m_epsilonX);
  iterBegin = mapUnicity->lower_bound(margeBasse);
  iterEnd = mapUnicity->upper_bound(margeHaute);

  //verification unicite du point de connexion
  for (it = iterBegin; it != iterEnd; ++it)
  {
      dataTmp = dm->getDataPointConnexion(it->second);
      BA_I(dataTmp != NULL);
      if ( dataTmp->getIdent() != m_data->getIdent() &&
           Util::floatCompare(fabs(dataTmp->getX()-m_data->getX()),ListDataCircuit::m_epsilonX)== Util::FLOAT_INF &&
           Util::floatCompare(fabs(dataTmp->getY()-m_data->getY()),ListDataCircuit::m_epsilonY)==Util::FLOAT_INF &&
           Util::floatCompare(fabs(Util::modulo2PI(dataTmp->getGamma()-m_data->getGamma())), ListDataCircuit::m_epsilonGamma)==Util::FLOAT_INF )
/*           Util::floatCompare(dataTmp->getX(),m_data->getX())==Util::FLOAT_EQU &&
           Util::floatCompare(dataTmp->getY(),m_data->getY())==Util::FLOAT_EQU &&
           Util::floatCompare(dataTmp->getGamma(),m_data->getGamma())==Util::FLOAT_EQU )*/
        {
          res = STATUS_ERROR;
          outMsg = "verification unicite : points de connexion '" +
                   m_data->getName() + "' et '" + dataTmp->getName() +
                   "' similaires";
          addInfoNotif(outMsg, res, m_data->getIdent());
          break;
        }
    }

  if (res == STATUS_ERROR)
  {
     return res;
  }

  listIdent = new ListIdent();
  dm->getListLink( m_idPtCon, listIdent);

  //verification de la coherence de la modif au niveau des troncons et + ...
  for (uint32_t i = 0; i<listIdent->size(); ++i)
    {
      idTrc = listIdent->getIdent(i);
      dataTrc = dm->getDataTroncon(idTrc);
      if (dataTrc != NULL)
        {
          algoTrc = new AlgoNotifyPtConOfTrc( idTrc,
                                              dataTrc->getIdDataPcDebut(),
                                              dataTrc->getIdDataPcFin(),
                                              dataTrc->getChoixConnecteurPcDebut(),
                                              dataTrc->getChoixConnecteurPcFin(),
                                              dm, getNoHorizontal()+1 );
          AlgoNotify::addAlgoNotify(algoTrc);
          res = algoTrc->doo();
          if (res == STATUS_ERROR)
            break;
        }
    }

  // bouger les bases associees si demande
  if ( res == STATUS_OK && m_depBase )
  {
      ListIdent *lstAcc = new ListIdent();
      AlgoListAccPc *algoListAccPc = new AlgoListAccPc(m_idPtCon, 0, lstAcc, dm);
      algoListAccPc->run(); // pas de verif...
      if ( lstAcc->size() != 0 )
      {
          for ( uint32_t index = 0; index < lstAcc->size(); index++)
          {
              ident_t idAcc = lstAcc->getIdent(index);
              DataAccostage *dataAcc = dm->getDataAccostage(idAcc);
              double xBase;
              double yBase;
              double gammaBase;

              AlgoCalcCoordBase *algo =
                  new AlgoCalcCoordBase(idAcc, &xBase, &yBase, &gammaBase, dm);
              status_t result = algo->run();
              delete algo;

              if ( dataAcc != NULL && result == STATUS_OK)
              {
                  ident_t idBase = dataAcc->getIdBase();
                  ListIdent *listNotChange = new ListIdent(); // delete dans destruction algoNotifyCoordOfBase

                  outMsg = "Modification de la base associee Id:" + Util::identToString(idBase);
                  addInfoNotif(outMsg, STATUS_OK, idBase);

                  // listNotChange->addIdent(idAcc);
                  AlgoNotifyCoordOfBase *algoNotifyCoordOfBase =
                      new AlgoNotifyCoordOfBase(idBase,
                       xBase, yBase, gammaBase,
                       listNotChange, dm, getNoHorizontal()+1);

                  AlgoNotify::addAlgoNotify(algoNotifyCoordOfBase);
                  res = algoNotifyCoordOfBase->doo();
                  getListCmpModified()->addIdentList(algoNotifyCoordOfBase->getListCmpModified());

                  delete listNotChange;
              }
          }
       }
       delete lstAcc;
  }

  if (res != STATUS_ERROR)
    {
      dm->changeSpecificUnicityList( static_cast<DataPointConnexion*>(m_dataSaved),
                                                      m_data);
      getListCmpModified()->addIdent(m_idPtCon);
    }
  delete listIdent;
  return res;
}

/** La modif ne peut etre conservee, il faut donc revenir en arriere
 *  . on doit remettre a jour les donnes du DataPointConnexion qui vont bien
 */
void
AlgoNotifyCoordOfPtCon::undoo()
{
  DataPointConnexion* dataSaved;

  if (m_dataSaved == NULL)
    return;

  dataSaved = static_cast<DataPointConnexion*>(m_dataSaved);
  getCmpDataManager()->changeSpecificUnicityList( m_data, dataSaved );
  //Mise a jour du DataPointConnexion
  *(m_data) = *dataSaved;

  getListCmpModified()->removeIdent(m_idPtCon);
}

/** Constructeur de l algo de modif d'alpha du point de connexion
 *  Met à jour le graph pour les tronçons limitrophes
 */
AlgoNotifyAlphaOfPtCon::AlgoNotifyAlphaOfPtCon( ident_t idPtCon,
                                                double alpha,
                                                CmpDataManager *cmpDataManager,
                                                uint32_t noHorizontal )
    : AlgoNotify(cmpDataManager, noHorizontal),
      m_idPtCon(idPtCon),
      m_alpha(alpha)
{
    DataPointConnexion* pcCon = getCmpDataManager()->getDataPointConnexion(m_idPtCon);
    if (pcCon == NULL)
    {
        BA_L(LOG_ERROR,"Modification de composant impossible");
    }
    else
    {
        m_dataSaved = pcCon->getCopy();
    }
    m_listTrc = new ListIdent();
};

/** Destructeur de l algo de modif d'alpha de Point de connexion
 *  Detruit les copie de sauvegarde du composant et de son data
 */
AlgoNotifyAlphaOfPtCon::~AlgoNotifyAlphaOfPtCon()
{
    if (m_dataSaved != NULL)
    {
        delete m_dataSaved;
        m_dataSaved = NULL;
    }
    if (m_listTrc != NULL)
    {
        delete m_listTrc;
        m_listTrc = NULL;
    }
}

/** Algorithme de notification de modification d'alaph de point de connexion
 *  La modification consiste en :
 *  . la mise à jour d'alpha
 *  . la mise à jour des graph pour les arcs limitrophes
 *  @param idPtCon identifiant du pt de connexion qu'on modifie
 *  @param alpha angle en radian (relatif)
 */
status_t
AlgoNotifyAlphaOfPtCon::doo()
{
    status_t res=STATUS_OK;
    ListIdent *listTrc2;
    DataPointConnexion* data;
    string outMsg;
    CmpTroncon* cmpTrc;
    CmpDataManager* dm = getCmpDataManager();
    AlgoListTrcAvecPcDebut* algoListTrcPcDeb;
    AlgoListTrcAvecPcFin* algoListTrcPcFin;

    m_majArcEffectuee = false;
    data = dm->getDataPointConnexion(m_idPtCon);
    m_listTrc->clear();
    if (data == NULL)
    {
        BA_L(LOG_ERROR,"Modification de composant impossible");
        return STATUS_ERROR;
    }

    //il y a au moins le troncon qui change, on fait comme si la distance changeait
    if (Util::floatCompare(m_alpha, data->getAlpha()) != Util::FLOAT_EQU)
    {
        data->setAlpha(m_alpha);

        //recherche des troncons impactes:
        algoListTrcPcDeb = new AlgoListTrcAvecPcDebut( m_idPtCon, m_listTrc, dm);
        listTrc2 = new ListIdent();
        algoListTrcPcFin = new AlgoListTrcAvecPcFin( m_idPtCon, listTrc2, dm);
        algoListTrcPcDeb->run();
        algoListTrcPcFin->run();
        delete algoListTrcPcDeb;
        delete algoListTrcPcFin;
        //on travaille avec m_listTrc
        m_listTrc->addIdentList(listTrc2);
        delete listTrc2;

        // bouger les bases associees
        AlgoListAccPc *algoListAccPc;
        ListIdent* lstBase, *listNotChange, *lstAccOfPc;
        lstAccOfPc = new ListIdent();
        lstBase = new ListIdent();
        listNotChange = new ListIdent();

        algoListAccPc = new AlgoListAccPc(m_idPtCon, 0, lstAccOfPc, dm);
        algoListAccPc->run(); // pas de verif...
        delete algoListAccPc;

        //parcoures des accostage du pc
        for ( uint32_t index = 0; index < lstAccOfPc->size(); index++)
        {
            ident_t idAcc = lstAccOfPc->getIdent(index);
            DataAccostage *dataAcc = dm->getDataAccostage(idAcc);
            double xBase;
            double yBase;
            double gammaBase;

            //accostage null: on sort
            if (dataAcc == NULL)
            {
                //pas le droit
                res = STATUS_ERROR;
                string outMsgNoAcc;
                outMsgNoAcc = "Modification alpha pc '"
                          + data->getName() + "' : accostage introuvable";
                addInfoNotif(outMsgNoAcc, res, data->getIdent());
                break;   
            }

            ident_t idBase = dataAcc->getIdBase();
            DataBase* dataBase = dm->getDataBase(idBase);

            if (dataBase->isLogique()) //pas besoin de modifier : on passe a l'accostage suivant
            {
                res = STATUS_OK;//rien
            }
            else
            {
                AlgoCalcCoordBase *algo =
                    new AlgoCalcCoordBase(idAcc, &xBase, &yBase, &gammaBase, dm);
                algo->run();
                delete algo;

                listNotChange->clear();
                listNotChange->addIdent(idAcc);
                outMsg = "Modification de la base associee Id:" + Util::identToString(idBase);
                addInfoNotif(outMsg, STATUS_OK, idBase);
                AlgoNotifyCoordOfBase *algoNotifyCoordOfBase =
                    new AlgoNotifyCoordOfBase(idBase,
                     xBase, yBase, gammaBase,
                     listNotChange, dm, getNoHorizontal()+1);
                AlgoNotify::addAlgoNotify(algoNotifyCoordOfBase);
                res = algoNotifyCoordOfBase->doo();
                getListCmpModified()->addIdentList(algoNotifyCoordOfBase->getListCmpModified());
            }
            if (res == STATUS_ERROR)
            {
                break;
            }
         }
         delete lstAccOfPc;
         delete lstBase;
         delete listNotChange;

        //on met le graph a jour
        if (res == STATUS_OK)
        {
            AlgoNotifyMarcheOfTrc* algoNotifyTrajectoireOfTrc;
            algoNotifyTrajectoireOfTrc = new AlgoNotifyMarcheOfTrc(0, false, false, getCmpDataManager(), getNoHorizontal()+1);
            for (uint32_t i = 0; i < m_listTrc->size(); ++i)
            {
                cmpTrc = dm->getCmpTroncon(m_listTrc->getIdent(i));
                algoNotifyTrajectoireOfTrc->majArcTrc(cmpTrc);
            }
            delete algoNotifyTrajectoireOfTrc;
            m_majArcEffectuee = true;
        }
    }

    if (res != STATUS_ERROR)
    {
        getListCmpModified()->addIdent(m_idPtCon);
        getListCmpModified()->addIdentList(m_listTrc);
        //on n'a rien changé, on ne met pas de lien a jour
        //getCmpDataManager()->updateLinkData( m_idPtCon );
    }
    return res;
}

/** La modif ne peut etre conservee, il faut donc revenir en arriere
 *  . on doit remettre le pointeur du DataTroncon qui va bien dans le composant
 *  . on doit remettre a jour les donnes du DataPointOrientation qui vont bien
 */
void
AlgoNotifyAlphaOfPtCon::undoo()
{
    DataPointConnexion* dataSaved, *dataPc;
    CmpTroncon* cmpTrc;
    CmpDataManager* dm;

    if (m_dataSaved == NULL)
        return;

    dm = getCmpDataManager();
    dataSaved = static_cast<DataPointConnexion*>(m_dataSaved);
    //Mise a jour du DataPointConnexion
    dataPc = dm->getDataPointConnexion(m_idPtCon);
    *dataPc = *dataSaved;

    if ( m_majArcEffectuee == true)
    {
        //maj du graph
        //on utilise cet algo, on aurait pu en prendre un autre. seul le service majArcTrc nous interesse
        AlgoNotifyMarcheOfTrc* algoNotifyTrajectoireOfTrc;
        algoNotifyTrajectoireOfTrc = new AlgoNotifyMarcheOfTrc(0, false, false, getCmpDataManager(), getNoHorizontal()+1);
        for (uint32_t i = 0; i < m_listTrc->size(); ++i)
        {
            cmpTrc = dm->getCmpTroncon(m_listTrc->getIdent(i));
            algoNotifyTrajectoireOfTrc->majArcTrc(cmpTrc);
        }
        delete algoNotifyTrajectoireOfTrc;
    }

    //Remise a jour des liens
    getListCmpModified()->removeIdent(m_idPtCon);
    for (uint32_t i=0; i < m_listTrc->size(); ++i)
    {
        getListCmpModified()->removeIdent(m_listTrc->getIdent(i));
    }
    //inutile : on n'a rien changé
    //getCmpDataManager()->updateLinkData( m_idPointOrientation );

}

/** Constructeur de l algo de modif de branche par Point de Connexion
 *  Fait une copie de sauvegarde du composant et de son data
 */
AlgoNotifyNumOfPointConnexion::AlgoNotifyNumOfPointConnexion
( ident_t idPointConnexion,
  uint32_t numPointConnexion,
  CmpDataManager *cmpDataManager,
  uint32_t noHorizontal )
  : AlgoNotify(cmpDataManager, noHorizontal),
    m_idPointConnexion(idPointConnexion),
    m_numPointConnexion(numPointConnexion)
{
  m_data = NULL;
  m_dataSaved = NULL;
  m_data = getCmpDataManager()->getDataPointConnexion(m_idPointConnexion);
  if (m_data == NULL)
    {
      BA_L(LOG_ERROR,"Modification de composant impossible");
    }
  else
    {
      m_dataSaved = m_data->getCopy();
    }
};

/** Destructeur de l algo de modif du numero Point de Connexion 
 *  Detruit les copie de sauvegarde du composant 
 */
AlgoNotifyNumOfPointConnexion::~AlgoNotifyNumOfPointConnexion()
{
  m_data = NULL;
  if (m_dataSaved != NULL)
    {
      delete m_dataSaved;
      m_dataSaved = NULL;
    }
}

/** Algorithme de notification de modification de numero de point de Connexion
 *  La modification consiste en :
 */
status_t
AlgoNotifyNumOfPointConnexion::doo()
{
  status_t res = STATUS_ERROR;
  ident_t idPointConnexion;
  string outMsg;
  ListIdent* list;

  if (m_data == NULL)
    return res;

  idPointConnexion = getCmpDataManager()->getIdPointConnexionByNum
    (m_data->getNumPointConnexion());
  if ( idPointConnexion == m_idPointConnexion )
    res = getCmpDataManager()->removeNumPointConnexion
      (m_data->getNumPointConnexion());
  else
    res = STATUS_ERROR;

  outMsg = "Suppression ancien numero du pt ctrl '" + m_data->getName() + "'(n°" +
           Util::identToString(m_data->getNumPointConnexion()) + ")" ;
  addInfoNotif(outMsg, res, m_data->getIdent());

  if (res == STATUS_OK)
    {
      res = getCmpDataManager()->addNumPointConnexion(m_numPointConnexion,
                                                     m_idPointConnexion);
      m_data->setNumPointConnexion(m_numPointConnexion);
    }

  outMsg = "Ajout nouveau numero du pt ctrl '" + m_data->getName()  + "'(n°" +
           Util::identToString(m_data->getNumPointConnexion()) + ")" ;
  addInfoNotif(outMsg, res, m_data->getIdent());

  list = new ListIdent();
  if (res != STATUS_ERROR)
    {
      getListCmpModified()->addIdent(m_idPointConnexion);
      //ajout des composants (brc et bases) qui appellent ce point de ctrl
      getCmpDataManager()->getListLink( m_idPointConnexion, list);
      getListCmpModified()->addIdentList( list);
    }
  delete list;
  return res;
}

/** La modif ne peut etre conservee, il faut donc revenir en arriere
 */
void
AlgoNotifyNumOfPointConnexion::undoo()
{
   DataPointConnexion *dataSaved;
   if (m_data!=NULL && m_dataSaved!=NULL)
     {
       dataSaved = static_cast<DataPointConnexion*>(m_dataSaved);

       //Mise a jour du DataPointConnexion et du numeor dans la liste
       m_data->setNumPointConnexion(dataSaved->getNumPointConnexion());
       getCmpDataManager()->addNumPointConnexion(m_data->getNumPointConnexion(), 
                                                m_idPointConnexion);
       
       getListCmpModified()->removeIdent(m_idPointConnexion);
     }
}

/** Constructeur de l algo de modif de branche par Point de Connexion
 *  Fait une copie de sauvegarde du composant et de son data
 */
AlgoNotifyOrientationOfPointConnexion::AlgoNotifyOrientationOfPointConnexion
( ident_t idPointConnexion,
        bool directOrientation,
        bool indirectOrientation,
        CmpDataManager *cmpDataManager,
        uint32_t noHorizontal )
  : AlgoNotify(cmpDataManager, noHorizontal),
    m_idPointConnexion(idPointConnexion),
    m_directOrientation  (directOrientation  ),
    m_indirectOrientation(indirectOrientation)
{
  m_data = NULL;
  m_dataSaved = NULL;
  m_data = getCmpDataManager()->getDataPointConnexion(m_idPointConnexion);
  m_listTrcModifie = new ListIdent();
  m_listIdAccDeleted = new ListIdent();
  if (m_data == NULL)
    {
      BA_L(LOG_ERROR,"Modification de composant impossible");
    }
  else
    {
      m_dataSaved = m_data->getCopy();
    }
};

/** Destructeur de l algo de modif du numero Point de Connexion
 *  Detruit les copie de sauvegarde du composant
 */
AlgoNotifyOrientationOfPointConnexion::~AlgoNotifyOrientationOfPointConnexion()
{
  m_data = NULL;
  if (m_dataSaved != NULL)
    {
      delete m_dataSaved;
      m_dataSaved = NULL;
    }
  if (m_listTrcModifie != NULL)
    {
      delete m_listTrcModifie;
      m_listTrcModifie = NULL;
    }
  if (m_listIdAccDeleted != NULL)
    {
      delete m_listIdAccDeleted;
      m_listIdAccDeleted = NULL;
    }
}

/** Algorithme de notification de modification de numero de point de Connexion
 *  La modification consiste en :
 */
status_t
AlgoNotifyOrientationOfPointConnexion::doo()
{
  status_t res = STATUS_OK;
  string outMsg;
  CmpTroncon* trc;
  CmpDataManager* dm = getCmpDataManager();

  if (m_data == NULL)
    return STATUS_ERROR;

  typedef enum {
	  NO_CHANGE_ORIENT = 0,
	  AUTORISE_ORIENT,
	  INTERDIT_ORIENT
  } changementOrient_t;


  changementOrient_t changementDirectOrient;
  if (m_directOrientation == m_data->getDirectOrientation())
	  changementDirectOrient = NO_CHANGE_ORIENT;
  else if (m_directOrientation == true)
	  changementDirectOrient = AUTORISE_ORIENT;
  else
	  changementDirectOrient = INTERDIT_ORIENT;
  changementOrient_t changementIndirectOrient;
  if (m_indirectOrientation == m_data->getIndirectOrientation())
	  changementIndirectOrient = NO_CHANGE_ORIENT;
  else if (m_indirectOrientation == true)
	  changementIndirectOrient = AUTORISE_ORIENT;
  else
	  changementIndirectOrient = INTERDIT_ORIENT;

  ListIdent listAgv;
  dm->getListIdData(&listAgv, DataManager::NAME_AGV);

  //on ajoute des autorisations, il faut ajouter l'accostage qui va bien (pour chaque type d'agv)
  if (changementDirectOrient == AUTORISE_ORIENT || changementIndirectOrient == AUTORISE_ORIENT)
  {
	  for (uint32_t i = 0; i < listAgv.size(); ++i)
	  {
		  ident_t idAgv = listAgv.getIdent(i);
		  ListIdent listAccTmp;
		  //rch des accostages sur ce point
		  AlgoAccOfPcAgv algoAccOfPcAgv( m_idPointConnexion, idAgv, &listAccTmp, dm);
		  algoAccOfPcAgv.run();

		  if (listAccTmp.size() == 0)
			  ; //rien, il n'y avait pas d'accostage, on n'en cree pas de nouveau
		  else
		  {
			  DataAccostage *dataAcc = dm->getDataAccostage(listAccTmp.getIdent(0));
			  //2 passages : le premier pour le direct et le second pour l'indirect
              for (uint32_t j = 0; j < 2; ++j)
              {
            	  DataAccostage::orientation_agv_t newOrientation;
            	  if (j == 0 && changementDirectOrient == AUTORISE_ORIENT)
            		  newOrientation = DataAccostage::DIRECTE;
            	  else if (j == 1 && changementIndirectOrient == AUTORISE_ORIENT)
            		  newOrientation = DataAccostage::INDIRECTE;
            	  else
            		  continue;

            	  AlgoNotifyAddAccostageOfBase * algoNotifyAddAccostageOfBase = new AlgoNotifyAddAccostageOfBase(dataAcc->getIdBase(), dataAcc->getIdPointConnexion(), newOrientation,
            			  dataAcc->getDecallageAgv(), dataAcc->getIdAgv(), dataAcc->getInitAssistee(), dataAcc->getBiberonage(), dataAcc->getIdBaseMotif(), dataAcc->getIndexMotif(), dm, getNoHorizontal()+1);

            	  res = algoNotifyAddAccostageOfBase->doo();
            	  if (res != STATUS_OK)
            	  {
            		  m_listInfoNotif->addMsgInfoList(algoNotifyAddAccostageOfBase->getListInfoNotif());
            		  goto fin;
            	  }
            	  getListCmpModified()->addIdentList(algoNotifyAddAccostageOfBase->getListCmpModified());
              }

              //code utile si on souhaite gerer le cote de la fleche : pas le cas pour les trajectoires symetriques
              //orientationAccOfConnecteurDirect = dataBase->getCoteZero() == DataBase::COTE_ZERO_DROITE ? DataAccostage::DIRECTE : DataAccostage::INDIRECTE;
			  //orientationAccOfConnecteurIndirect = dataBase->getCoteZero() == DataBase::COTE_ZERO_DROITE ? DataAccostage::INDIRECTE : DataAccostage::DIRECTE;
		  }
	  }
  }

  m_listIdAccDeleted->clear();
  //on supprime des autorisations, il faut supprimer les accostages existants (pour chaque type d'agv)
  if (changementDirectOrient == INTERDIT_ORIENT || changementIndirectOrient == INTERDIT_ORIENT)
  {
	  for (uint32_t i = 0; i < listAgv.size(); ++i)
	  {
		  ident_t idAgv = listAgv.getIdent(i);
		  ListIdent listAccTmp;
		  //rch des accostages sur ce point
		  AlgoAccOfPcAgv algoAccOfPcAgv( m_idPointConnexion, idAgv, &listAccTmp, dm);
		  algoAccOfPcAgv.run();

		  for (uint32_t j = 0; j < listAccTmp.size(); ++j)
		  {
			  DataAccostage *dataAcc = dm->getDataAccostage(listAccTmp.getIdent(j));
			  DataAccostage::orientation_agv_t orientationAccOfConnecteurDirect = DataAccostage::DIRECTE;
			  DataAccostage::orientation_agv_t orientationAccOfConnecteurIndirect = DataAccostage::INDIRECTE;
              //code utile si on souhaite gerer le cote de la fleche : pas le cas pour les trajectoires symetriques
			  //DataBase *dataBase = dm->getDataBase(dataAcc->getIdBase());
			  //DataAccostage::orientation_agv_t orientationAccOfConnecteurDirect = dataBase->getCoteZero() == DataBase::COTE_ZERO_DROITE ? DataAccostage::DIRECTE : DataAccostage::INDIRECTE;
			  //DataAccostage::orientation_agv_t orientationAccOfConnecteurIndirect = dataBase->getCoteZero() == DataBase::COTE_ZERO_DROITE ? DataAccostage::INDIRECTE : DataAccostage::DIRECTE;
			  if ( (changementDirectOrient == INTERDIT_ORIENT && dataAcc->getOrientationAgv() == orientationAccOfConnecteurDirect) ||
					  (changementIndirectOrient == INTERDIT_ORIENT && dataAcc->getOrientationAgv() == orientationAccOfConnecteurIndirect) )
			  {
				  AlgoNotifyRemoveAccostageOfBase* algoNotifyRemoveAccostageOfBase = new AlgoNotifyRemoveAccostageOfBase( dataAcc->getIdBase(), dataAcc->getIdent(), m_listInfoNotif, dm, getNoHorizontal()+1 );
				  AlgoNotify::addAlgoNotify(algoNotifyRemoveAccostageOfBase);
				  res = algoNotifyRemoveAccostageOfBase->doo();
				  getListCmpModified()->addIdentList(algoNotifyRemoveAccostageOfBase->getListCmpModified());
				  m_listIdAccDeleted->addIdent(listAccTmp.getIdent(j));
				  if (res != STATUS_OK)
					  goto fin;
			  }
		  }
	  }
  }

  //modificiation du graph
  if (res == STATUS_OK)
  {
	  //setData
	  dm->setPenalisationAJour(false);
	  m_data->setDirectOrientation(m_directOrientation);
	  m_data->setIndirectOrientation(m_indirectOrientation);
	  getListCmpModified()->addIdent(m_idPointConnexion);

	  //recherche des troncons utilisant ce point de connexion
	  m_listTrcModifie->clear();
	  AlgoListTrcLiePc algoListTrcLiePc( m_idPointConnexion, m_listTrcModifie, dm );
	  res = algoListTrcLiePc.run();
	  if ( res == STATUS_OK)
	  {
		  for (uint32_t i = 0; i < m_listTrcModifie->size(); ++i)
		  {
			  trc = dm->getCmpTroncon( m_listTrcModifie->getIdent(i));
			  AlgoCheckSymmetricalTrc algoNotifyTrajectoireOfTrc(trc->getIdent(), dm, getNoHorizontal()+1); //entourloupe pour appeler la classe abstraite algoNotifyTrajectoireOfTrc
			  algoNotifyTrajectoireOfTrc.updateGraph( trc );
			  getListCmpModified()->addIdentList(m_listTrcModifie);
		  }
	  }
  }

  fin:
  return res;
}

/** La modif ne peut etre conservee, il faut donc revenir en arriere
 */
void
AlgoNotifyOrientationOfPointConnexion::undoo()
{
   DataPointConnexion *dataSaved;
   CmpTroncon* trc;
   CmpDataManager* dm = getCmpDataManager();

   if (m_data!=NULL && m_dataSaved!=NULL)
     {
       dataSaved = static_cast<DataPointConnexion*>(m_dataSaved);
       *m_data = *dataSaved;

       for (uint32_t i = 0; i < m_listTrcModifie->size(); ++i)
       {
           trc = dm->getCmpTroncon( m_listTrcModifie->getIdent(i));
           AlgoCheckSymmetricalTrc algoNotifyTrajectoireOfTrc(trc->getIdent(), dm, getNoHorizontal()+1); //entourloupe pour appeler la classe abstraite algoNotifyTrajectoireOfTrc
           algoNotifyTrajectoireOfTrc.updateGraph( trc );
           getListCmpModified()->removeIdent(trc->getIdent());
       }
       getListCmpModified()->removeIdent(m_idPointConnexion);
     }
}


// ------ FIN -----------------------------------------------------------------




