/* BA Systemes
 *
 * Projet: 0000Z - SPEED
 * Composants: C_CIRCUIT_lASER-01
 * Rajkrishna Paul 
 * __IDCVS__
 */

// ----------------------------------------------------------------------------

/** @file
 *  Objet algos intersection.
 */

// ------ DI - directives d'inclusion -----------------------------------------
#include "track/lib/algo_intersections.h"
#include "track/lib/algo_cmp_agv.h"
#include "track/lib/tube.h"

#ifdef __WIN32__
#include <math.h>
#endif
#include <cstdlib>

// ------ CP - constantes privees ---------------------------------------------

// ------ MP - macros privees -------------------------------------------------

// ------ TP - types et structures prives -------------------------------------

// ------ VG - variables globales publiques et privees ------------------------

// ------ PF - prototypes des fonctions privees -------------------------------

// ------ CF - code des methodes ----------------------------------------------

//---------------------------------------------------------------------------
/* Algo permettant de calculer les points du polygone d'un AGV, d'une porte...
 * l'algo met à jour un vecteur de pointGeom_t
 */
status_t
AlgoCalculPolygone::run()
{
    pointGeom_t p1, p2, p3, p4, p14, p23;
    double cosGamma, sinGamma;
    double largeurGaucheCosGamma, largeurGaucheSinGamma, largeurDroiteCosGamma, largeurDroiteSinGamma;

    //            Gauche
    //   P1-------------------P2
    //    |                   |
    //    |                   |
    // Ar P14     Milieu      P23 Av
    //    |                   |
    //    |                   |
    //   P4-------------------P3
    //            Droite

    cosGamma = cos( m_gamma);
    sinGamma = sin( m_gamma);
    largeurGaucheCosGamma = m_yGauche*cosGamma;
    largeurGaucheSinGamma = m_yGauche*sinGamma;
    largeurDroiteCosGamma = m_yDroit*cosGamma;
    largeurDroiteSinGamma = m_yDroit*sinGamma;

    //calcul des coordonnees de P14 (arriere)
    p14.px = m_x + m_xAr*cosGamma;
    p14.py = m_y + m_xAr*sinGamma;

    //calcul des coordonnees de P23 (avant)
    p23.px = m_x + m_xAv*cosGamma;
    p23.py = m_y + m_xAv*sinGamma;

    //autres points arrieres
    p1.px = p14.px - largeurGaucheSinGamma;
    p1.py = p14.py + largeurGaucheCosGamma;

    p4.px = p14.px - largeurDroiteSinGamma;
    p4.py = p14.py + largeurDroiteCosGamma;

    //autres points avant
    p2.px = p23.px - largeurGaucheSinGamma;
    p2.py = p23.py + largeurGaucheCosGamma;

    p3.px = p23.px - largeurDroiteSinGamma;
    p3.py = p23.py + largeurDroiteCosGamma;

    m_polygone->resize(4);
    (*m_polygone)[0] = p1;
    (*m_polygone)[1] = p2;
    (*m_polygone)[2] = p3;
    (*m_polygone)[3] = p4;

    return STATUS_OK;
}

void
AlgoCalculPolygone::setValues( vector<pointGeom_t>* polygone,
                               double x, double y, double gamma,
                               double xAv, double xAr,
                               double yGauche, double yDroit )
{
    m_polygone = polygone;
    m_x = x;
    m_y = y;
    m_gamma = gamma;
    m_xAv = xAv;
    m_xAr = xAr;
    m_yGauche = yGauche;
    m_yDroit = yDroit;
}

//---------------------------------------------------------------------------
/**
 * set des parametres de l'algo
 */
void
AlgoIntersectionPairePolygone::setValues( const vector<pointGeom_t> *poly1,
                                          const vector<pointGeom_t> *poly2,
                                          vector<pointGeom_t> *res,
                                          bool finirVite )
{
    m_poly1 = poly1;
    m_poly2 = poly2;
    m_res = res;
    m_finirVite = finirVite;
}

//---------------------------------------------------------------------------
/* algo permettant de calculer les intersections entre 2 polygones
 * l'algo met a jour une structure m_res
*/
status_t AlgoIntersectionPairePolygone::run()
{
    m_res->clear();
    for ( vector<pointGeom_t>::const_iterator i = m_poly1->begin(); i != m_poly1->end(); ++i )
    {
        const pointGeom_t *pointSuivantI;
        if(i!=m_poly1->end()-1)
        {
            pointSuivantI = &*(i+1);
        }
        else
        {
            pointSuivantI = &*(m_poly1->begin());
        }
        for ( vector<pointGeom_t>::const_iterator j = m_poly2->begin(); j != m_poly2->end(); ++j )
        {
            double xRes,yRes;
            const pointGeom_t *pointSuivantJ;
            if(j!=m_poly2->end()-1)
            {
                pointSuivantJ = &*(j+1);
            }
            else
            {
                pointSuivantJ = &*(m_poly2->begin());
            }
            int inter = Intersection::calculIntersectionSegmentDroiteSegmentDroite(
                    (*i).px,(*i).py,(*pointSuivantI).px,(*pointSuivantI).py,
                    (*j).px,(*j).py,(*pointSuivantJ).px,(*pointSuivantJ).py,
                    &xRes, &yRes);
            if(inter==1)
            {
                #if GROS_DEBUG_CRF
                DEBUG_POINT(xRes,yRes);
                #endif
                pointGeom_t pointCourant;
                pointCourant.px = xRes;
                pointCourant.py = yRes;
                m_res->push_back(pointCourant);
                if(m_finirVite) return STATUS_OK;
            }
        }
    }
    for ( vector<pointGeom_t>::const_iterator i = m_poly1->begin(); i != m_poly1->end(); ++i )
    {
        bool resSpace;
        AlgoPointIsInSpace algoCoin((*i).px,(*i).py,m_poly2,&resSpace,getCmpDataManager());
        algoCoin.run();
        if(resSpace)
        {
            #if GROS_DEBUG_CRF
            DEBUG_POINT((*i).px,(*i).py);
            #endif
            pointGeom_t pointCourant;
            pointCourant.px = (*i).px;
            pointCourant.py = (*i).py;
            m_res->push_back(pointCourant);
            if(m_finirVite) return STATUS_OK;
        }
    }
    for ( vector<pointGeom_t>::const_iterator j = m_poly2->begin(); j != m_poly2->end(); ++j )
    {
        bool resSpace;
        AlgoPointIsInSpace algoCoin((*j).px,(*j).py,m_poly1,&resSpace,getCmpDataManager());
        algoCoin.run();
        if(resSpace)
        {
            #if GROS_DEBUG_CRF
            DEBUG_POINT((*j).px,(*j).py);
            #endif
            pointGeom_t pointCourant;
            pointCourant.px = (*j).px;
            pointCourant.py = (*j).py;
            m_res->push_back(pointCourant);
            if(m_finirVite) return STATUS_OK;
        }
    }
    return STATUS_OK;
}


/** remplit un booleen pour specifier si le point appartient ou non a l espace
 *  ( Jordan curve theorem :
 *  http://www.ecse.rpi.edu/Homepages/wrf/research/geom/pnpoly.html )
 */
status_t
AlgoPointIsInSpace::run()
{
  vector<pointGeom_t>::const_iterator iterI, iterJ;

  *m_isInSpace = false;

  for (iterI = m_lstPointGeom->begin(), iterJ = m_lstPointGeom->end()-1; iterI != m_lstPointGeom->end(); iterJ = iterI++)
  {
      const double xI = (*iterI).px;
      const double yI = (*iterI).py;
      const double xJ = (*iterJ).px;
      const double yJ = (*iterJ).py;

#define NO_FLOAT_COMPARE
#ifdef NO_FLOAT_COMPARE
      if ( ( ( yI<= m_y && m_y < yJ ) || ( yJ <= m_y && m_y< yI ) )
    		  && ( m_x < (xJ - xI)*(m_y - yI) / (yJ - yI) + xI) )
#else
      if ( ( ( (Util::floatCompare(yI, m_y) != Util::FLOAT_SUP) &&
               (Util::floatCompare(m_y, yJ) == Util::FLOAT_INF) ) ||
             ( (Util::floatCompare(yJ, m_y) != Util::FLOAT_SUP) &&
               (Util::floatCompare(m_y, yI) == Util::FLOAT_INF) ) ) &&
           ( Util::floatCompare(m_x, (xJ - xI)*(m_y - yI) / (yJ - yI) + xI) ==
                        Util::FLOAT_INF ) )

#endif
      {
        *m_isInSpace = !(*m_isInSpace);
      }
  }

  return STATUS_OK;
}

void
AlgoPointIsInSpace::setValues(double x, double y, const vector<pointGeom_t>* lstPointGeom, bool* isInSpace)
{
  m_x = x;
  m_y = y;
  m_lstPointGeom = lstPointGeom;
  m_isInSpace = isInSpace;
}

/*
  int pnpoly(int npol, float *xp, float *yp, float x, float y)
  {
  int i, j, c = 0;
  for (i = 0, j = npol-1; i < npol; j = i++) {
  if ((((yp[i]<=y) && (y<yp[j])) ||
  ((yp[j]<=y) && (y<yp[i]))) &&
  (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))

  c = !c;
  }
  return c;
  }
*/

void
AlgoIntersectionAgv::setValues(double x1, double y1, double theta1, ident_t idAgv1, bool isTheoric1,
                               double x2, double y2, double theta2, ident_t idAgv2, bool isTheoric2,
                               bool *choc)
{
    m_x1 = x1;
    m_y1 = y1;
    m_theta1 = theta1;
    m_idAgv1 = idAgv1;
    m_isTheoric1 = isTheoric1;
    m_x2 = x2;
    m_y2 = y2;
    m_theta2 = theta2;
    m_idAgv2 = idAgv2;
    m_isTheoric2 = isTheoric2;
    m_choc = choc;
}

/* algo permettant de calculer les points d'intersections entre deux AGV
 *
*/
status_t AlgoIntersectionAgv::run()
{
    vector<pointGeom_t> v1, v2, vRes;
    pointGeom_t p;
    CmpDataManager *dm = getCmpDataManager();
    int32_t distSecu1 = dm->getDistSecu(m_isTheoric1);
    int32_t distSecu2 = dm->getDistSecu(m_isTheoric2);
    DataAgv *agv1, *agv2;

    //init
    agv1= dm->getDataAgv(m_idAgv1);
    if(agv1 == NULL)
    {
        return STATUS_ERROR;
    }
    double dav1 = agv1->getXAvGabaritAgv() + distSecu1;
    double dar1 = agv1->getXArGabaritAgv() - distSecu1;
    double dg1 = agv1->getYGaucheGabaritAgv() + distSecu1;
    double dd1 = agv1->getYDroiteGabaritAgv() - distSecu1;

    agv2 = dm->getDataAgv(m_idAgv2);
    if(agv2 == NULL)
    {
        return STATUS_ERROR;
    }
    double dav2 = agv2->getXAvGabaritAgv() + distSecu2;
    double dar2 = agv2->getXArGabaritAgv() - distSecu2;
    double dg2 = agv2->getYGaucheGabaritAgv() + distSecu2;
    double dd2 = agv2->getYDroiteGabaritAgv() - distSecu2;

    //discrimination
    double dDiscr = dm->getRayonAgvPhysique(m_idAgv1, m_isTheoric1) +
                    dm->getRayonAgvPhysique(m_idAgv2, m_isTheoric2);
    if(sqrt((m_x1-m_x2)*(m_x1-m_x2)+(m_y1-m_y2)*(m_y1-m_y2)) > dDiscr)
    {
        *m_choc = false;
        return STATUS_OK;
    }

    //AGV1
    AlgoCalculPolygone algoPolygone(&v1, m_x1, m_y1, m_theta1, dav1, dar1, dg1, dd1, dm);
    algoPolygone.run();

    //AGV2
    algoPolygone.setValues(&v2, m_x2, m_y2, m_theta2, dav2, dar2, dg2, dd2);
    algoPolygone.run();

    AlgoIntersectionPairePolygone algo(&v1, &v2, &vRes, true, dm);
    algo.run();

    if(vRes.size() > 0)
        *m_choc = true;
    else
        *m_choc = false;

    return STATUS_OK;
}


void
AlgoIntersectionAgvSansDiscrimination::setValues( double x1, double y1, double theta1, ident_t idAgv1, bool isTheoric1,
                                                  double x2, double y2, double theta2, ident_t idAgv2, bool isTheoric2,
                                                  bool *choc)
{
    m_x1 = x1;
    m_y1 = y1;
    m_theta1 = theta1;
    m_idAgv1 = idAgv1;
    m_isTheoric1 = isTheoric1;
    m_x2 = x2;
    m_y2 = y2;
    m_theta2 = theta2;
    m_idAgv2 = idAgv2;
    m_isTheoric2 = isTheoric2;
    m_choc = choc;
}

/* algo permettant de calculer les points d'intersections entre deux AGV
 *
*/
status_t AlgoIntersectionAgvSansDiscrimination::run()
{
    vector<pointGeom_t> v1, v2, vRes;
    pointGeom_t p;
    CmpDataManager *dm = getCmpDataManager();
    int32_t distSecu1 = dm->getDistSecu(m_isTheoric1);
    int32_t distSecu2 = dm->getDistSecu(m_isTheoric2);

    //AGV1
    DataAgv *agv = dm->getDataAgv(m_idAgv1);
    AlgoCalculPolygone algoPolygone( &v1,
                                     m_x1, m_y1, m_theta1,
                                     agv->getXAvGabaritAgv() + distSecu1,
                                     agv->getXArGabaritAgv() - distSecu1,
                                     agv->getYGaucheGabaritAgv() + distSecu1,
                                     agv->getYDroiteGabaritAgv() - distSecu1,
                                     dm );
    algoPolygone.run();

    //AGV2
    agv = dm->getDataAgv(m_idAgv2);
    algoPolygone.setValues( &v2,
                            m_x2, m_y2, m_theta2,
                            agv->getXAvGabaritAgv() + distSecu2,
                            agv->getXArGabaritAgv() - distSecu2,
                            agv->getYGaucheGabaritAgv() + distSecu2,
                            agv->getYDroiteGabaritAgv() - distSecu2 );
    algoPolygone.run();

    AlgoIntersectionPairePolygone algo(&v1, &v2, &vRes, true, dm);
    algo.run();
    if(vRes.size() > 0)
        *m_choc = true;
    else
        *m_choc = false;

    return STATUS_OK;
}

void
AlgoListPointTrc::setValues(ident_t idTrc, bool marcheAv, ident_t idAgv, listPointTube_t *listPoint,
  double s, double longueurOffset)
{
    m_idTrc = idTrc;
    m_marcheAv = marcheAv;
    m_idAgv = idAgv;
    m_listPoint = listPoint;
    m_s = s;
    m_longueurOffset = longueurOffset;
}

/* algo permettant de calculer une liste de points correspondant a un troncon
 * ATTENTION 1 : Cet Algo ne vide pas la liste qui lui est passee en parametre
 * ATTENTION 2 : Cet Algo cree des points tube (new), les met dans une liste, ce n'est pas lui qui les detruit !
*/
status_t AlgoListPointTrc::run()
{
    CmpDataManager *dm = getCmpDataManager();
    CmpTroncon *trc = dm->getCmpTroncon(m_idTrc);
    DataAgv *agv = dm->getDataAgv(m_idAgv);
    int32_t distStep = dm->getDataInfoGlobales()->getPasDiscretisationAnticonflit();
    if(trc == NULL || agv==NULL)
    {
        return STATUS_ERROR;
    }
    double lTot = trc->getLength();
    PointTube *pCourbe;
    PointCircuit p(0, PointCircuit::AUTRE, 0, 0, 0 );
    double s = m_longueurOffset;
    if (!m_marcheAv)
    {
      s = lTot;
    }
    while (((s<lTot+distStep) && m_marcheAv) || ((s>-distStep) && !m_marcheAv))
    {
        dm->getServicesInfoTrajectoire()->calculerPointSurTroncon(trc, agv, s, &p);
        pCourbe = new PointTube(p.getX(), p.getY(), p.getGamma() + p.getAlpha());
        pCourbe->setIdTrc(m_idTrc);
        pCourbe->setMarcheAv(m_marcheAv);
        if (m_marcheAv)
        {
          pCourbe->setS(min(s + m_s, m_s + lTot));
        }
        else
        {
          pCourbe->setS(min(lTot - s + m_s, m_s + lTot));
        }
        m_listPoint->push_back(pCourbe);
        if (m_marcheAv)
        {
          s += distStep;
        }
        else
        {
          s -= distStep;
        }
    }

    return STATUS_OK;
}

/**
 * algo permettant de calculer les tronçons sous un point
 */
status_t AlgoIntersectionTrc::run()
{
	status_t res=STATUS_ERROR;
	CmpDataManager *dm = getCmpDataManager();
	if(dm)
	{
		PointCircuit point;
		//On parcours tous les tronçons
		for(uint32_t i=0 ; i<dm->getNbDataTroncon() ; i++)
		{
			CmpTroncon *trc = dm->getCmpTronconByIndex(i);
            if(!trc->getDataTroncon()->getToBeRecorded())
            {
            	continue;// On omet les troncons non enregistrables
            }

			ServicesInfoTrajectoire::info_point_troncon_s infoPointTroncon;
			//Calcul des distances aux PC du troncon
			const double dDeb = sqrt(Util::carre(trc->getDataPointConnexionDebut()->getX() - m_x) +
					Util::carre(trc->getDataPointConnexionDebut()->getY() - m_y));
			const double dFin = sqrt(Util::carre(trc->getDataPointConnexionFin()->getX() - m_x) +
					Util::carre(trc->getDataPointConnexionFin()->getY() - m_y));

			if((dDeb < trc->getLength() + m_radius)	&& (dFin < trc->getLength() + m_radius)  //Premier filtre, distance min au troncon < tailleTroncon + radius
					&& dm->getServicesInfoTrajectoire()->calculerPointSurTroncon(trc, m_dataAgv, m_x, m_y, &point, &infoPointTroncon, NULL) == STATUS_OK  //on calcule le point sur le troncon
					&& (Vector2(m_x,m_y)-Vector2(point.getX(),point.getY())).length() < m_radius) //Si ce point est à la bonne distance, on a notre tronçon
			{
				 m_listTrc.addIdent(trc->getIdent());
			}
		}// for all trc
		res = STATUS_OK;
	}// if dm
	return res;
}


/* algo permettant de calculer les troncons genes par la presence d'un AGV
 *
*/
status_t AlgoIntersectionAgvTrc::run()
{
    status_t res=STATUS_OK;
    double dDeb, dFin, lengthSup;
    CmpDataManager *dm = getCmpDataManager();
    CmpTroncon *trc;
    AlgoListPointTrc algoTrc(0, 0, NULL, dm);
    listPointTube_t listPoint;
    bool choc;
    AlgoIntersectionAgv algoAgv(0.0, 0.0, 0.0, 0, false, 0.0, 0.0, 0.0, 0, true, NULL, dm);

    double rayonAgv = dm->getRayonAgvPhysique(m_idAgv, false);
    double rayonAgvCible = dm->getRayonAgvPhysique(m_idAgvCible, true);

    m_listTrc.clear();

    if(rayonAgv == 0.0 || rayonAgvCible == 0.0)
    {
        res = STATUS_ERROR;
    }
    lengthSup = rayonAgv + rayonAgvCible;

    for(uint32_t i=0 ; res == STATUS_OK && i<dm->getNbDataTroncon() ; i++)
    {
        trc = dm->getCmpTronconByIndex(i);
        //discrimination
        dDeb = sqrt(Util::carre(trc->getDataPointConnexionDebut()->getX() - m_x) +
                    Util::carre(trc->getDataPointConnexionDebut()->getY() - m_y));
        dFin = sqrt(Util::carre(trc->getDataPointConnexionFin()->getX() - m_x) +
                    Util::carre(trc->getDataPointConnexionFin()->getY() - m_y));

        if((dDeb < trc->getLength() + lengthSup) && (dFin < trc->getLength() + lengthSup))
        {
            //troncon potentiellement impacte par notre AGV, verifions !
            listPoint.clear();
            algoTrc.setValues(trc->getIdent(), true, m_idAgvCible, &listPoint);
            //attention, les points doivent etre supprimes un a un !!!
            algoTrc.run();
            choc = false;
            for(listPointTube_t::iterator it = listPoint.begin() ;
                it != listPoint.end() ; it++)
            {
                algoAgv.setValues(m_x, m_y, m_theta, m_idAgv, false,
                                  (*it)->getX(), (*it)->getY(), (*it)->getTheta(), m_idAgvCible, true,
                                  &choc);
                algoAgv.run();
                if(choc)
                {
                    break;
                }
            }
            for(listPointTube_t::iterator it = listPoint.begin() ;
                it != listPoint.end() ; it++)
            {
                delete (*it);
            }
            if(choc)
            {
                m_listTrc.addIdent(trc->getIdent());
            }
        }
    }

    return res;
}

/* Calcule l'intersection entre 2 points cibles
 * @return STATUS_ERROR si erreur dans intersection paire polygone (donc jamais)
 * @return STATUS_OK tout va bien
 * m_intersection est renseignee (vrai ou faux)
 */
status_t AlgoIsIntersectionPointTube::run()
{
    double rayonProprioPhysique = m_ptProprio->getRayonAgvPhysique();
    double rayonProprioSecu = m_ptProprio->getRayonAgvChampSecu();

    double rayonCiblePhysique = m_ptCible->getRayonAgvPhysique();
    double rayonCibleSecu = m_ptCible->getRayonAgvChampSecu();

    *m_isIntersection = false;
    *m_distanceCarre = Util::carre(m_ptProprio->getX()-m_ptCible->getX()) + Util::carre(m_ptProprio->getY()-m_ptCible->getY());
    if( *m_distanceCarre > Util::carre(rayonProprioPhysique + rayonCibleSecu) &&
    		*m_distanceCarre > Util::carre(rayonProprioSecu + rayonCiblePhysique) )
    {
    	//pas la peine de faire le calcul exact, il n'y a pas d'intersection
    	return false;
    }

    *m_isIntersection = m_ptProprio->isInteractionPhysiqueVsChampSecu(*m_ptCible)
    						||
    					m_ptProprio->isInteractionChampSecuVsPhysique(*m_ptCible)
    						||
    					m_ptProprio->isInteractionPhysiqueVsPhysique(*m_ptCible);

    return STATUS_OK;
}

void
AlgoIsIntersectionPointTube::setValues(PointTube *ptProprio,
                                       PointTube *ptCible,
                                       bool *isIntersection,
                                       double* distCarre)
{
    m_ptProprio = ptProprio;
    m_ptCible = ptCible;
    m_isIntersection = isIntersection;
    m_distanceCarre = distCarre;
}



/** Calcule un point tube e des coordonnees
 *  Le principal role est de calculer le polygone AGV du point tube
 * @return STATUS_ERROR si le pointTube est nul
 * @return STATUS_WARNING si le dataAgv est nul
 * @return STATUS_OK si tout va bien (on a reussi a remplir le polygone)
 */
status_t AlgoCalculPointTube::run()
{

    PointTube* pointTube = m_algoChangeVitessePointTube.getPointTube();
    DataAgv* dataAgv =  m_algoChangeVitessePointTube.getDataAgv();

    if( pointTube == NULL )
    {
        return STATUS_ERROR;
    }

    pointTube->razAllSeq();

    OBB2D rectPhysique(pointTube->getRectanglePhysique());

    if( dataAgv != NULL )
    {
    	const double  distSecu = getCmpDataManager()->getDistSecu( m_isPointTubeTheoric );

        //---------------------------------------------------------------------
        // recalcul du polygone physique
        const double dav = dataAgv->getXAvGabaritAgv() + distSecu;
        const double dar = dataAgv->getXArGabaritAgv() - distSecu;
        const double dg = dataAgv->getYGaucheGabaritAgv() + distSecu;
        const double dd = dataAgv->getYDroiteGabaritAgv() - distSecu;
        const double halfHeigth = (dg-dd)/2;
        const double halfWidth = (dav-dar)/2;
        const double centreX = m_x + cos(m_theta) * (dav - halfWidth) - sin(m_theta) * (dg - halfHeigth);
        const double centreY = m_y + sin(m_theta) * (dav - halfWidth) + cos(m_theta) * (dg - halfHeigth);

        rectPhysique.setValues(centreX, centreY, halfWidth, halfHeigth, m_theta);
    }

    pointTube->resetPosData( m_x,
                             m_y,
                             m_theta,
                             m_marcheAv,
                             m_isPointTubeTheoric,
                             rectPhysique );

    return m_algoChangeVitessePointTube.run();
}


void
AlgoCalculPointTube::setValues( DataAgv* dataAgv,
                                double x,
                                double y,
                                double theta,
                                bool marcheAv,
                                int16_t vitesse,
                                bool checkZoneRalenti,
                                bool isPointTubeTheoric,
                                PointTube *pointTube,
                                const GeomChampSecurite& champSecuMini)
{
    m_x = x;
    m_y = y;
    m_theta = theta;
    m_marcheAv = marcheAv;
    m_isPointTubeTheoric = isPointTubeTheoric;

    m_algoChangeVitessePointTube.setValues(dataAgv,
                                      vitesse,
                                      0,
                                      checkZoneRalenti,
                                      pointTube,
                                      champSecuMini);
}

/** Recalcule la partie "vitesse" d'un point tube
 *  Le principal role est de calculer le polygone des champs de securite du point tube
 * @return STATUS_ERROR si le pointTube est nul
 * @return STATUS_WARNING si le dataAgv est nul
 * @return STATUS_OK si tout va bien (on a reussi a remplir le polygone)
 */
status_t AlgoChangeVitessePointTube::run()
{
    status_t res = STATUS_WARNING;
    double distSecu;
    GeomChampSecurite geomChampSecuTmp, newGeomChampSecu;
    double debordMax = 0.0;
    bool useS100 = true;
    bool alleeEtroite = false;

    if (m_pointTube == NULL || m_dataAgv == NULL)
    {
        res = STATUS_ERROR;
        return res;
    }

    {
    	// Variables mises a jour par l'appel de l'algo zone.
    	//Seules les variables requises dans le reste de la methodes sont sorties de la portee reduite.
    	double ralenti = 0.0;
    	bool bipNormalActif = false, bipFortActif = false, chauffageScrutateurSecu = false, entreePlsB = false, inhibitionS100 = false;
    	uint32_t hauteurMaxFourches = 0;

    	AlgoAgvOptionsZone algoAgvOptionsZone(
    			m_dataAgv->getIdent(),
				m_pointTube->getX(),
				m_pointTube->getY(),
				m_pointTube->getTheta(),
				m_pointTube->isTheoric(),
    			&ralenti,
				&bipNormalActif,
				&bipFortActif,
				&chauffageScrutateurSecu,
				&entreePlsB,
				&inhibitionS100,
				&alleeEtroite,
				&hauteurMaxFourches,
				NULL, // Liste des zones spécifiques
				getCmpDataManager() );
    	status_t resAlgo = algoAgvOptionsZone.run();

    	// L'AGV est a l'interieur d'une zone de ralenti
    	if( ralenti != 0.0 && abs(m_vitesse) > static_cast<int16_t>(ralenti) )
    	{
    		m_vitesse = ((m_vitesse >= 0) ? static_cast<int16_t>(ralenti) : -static_cast<int16_t>(ralenti));
    	}

    	// L'AGV est a l'interieur d'une zone d'inhibition du S100
    	useS100 = ( resAlgo != STATUS_OK || !inhibitionS100 );
    }

    // On ne calcule PAS de nouveaux champs de secu
    // - si l'AGV ne possède pas de champ de secu
    // - si l'AGV est un AIV et se trouve dans une zone allee etroite
    if(( m_dataAgv->getTypeAgv() == DataAgv::AIV && alleeEtroite ) || getCmpDataManager()->getNbCmpChampSecurite() == 0)
    {
        m_pointTube->changeVitesse( m_vitesse,
                                m_vitesseCurrentComputation,
                                NULL );
    }
    // Sinon on calcule les nouveaux champs de secu
    else
    {
        distSecu = getCmpDataManager()->getDistSecu( m_pointTube->isTheoric() );

        // recuperation des champs de securite
        for( int typeChampSecu = 0;
             DataChampSecurite::type_champ_secu_t(typeChampSecu) < DataChampSecurite::NB_CHAMP_SECU;
             ++typeChampSecu )
        {
        	// les champs de securite de type CHAMP_DOCKING_STATION ne sont pas à considérer par la supervision
        	if( DataChampSecurite::type_champ_secu_t(typeChampSecu) == DataChampSecurite::CHAMP_DOCKING_STATION )
                continue;

            if( !useS100 && DataChampSecurite::type_champ_secu_t(typeChampSecu) == DataChampSecurite::CHAMP_S100 )
                continue;

            if( !getCmpDataManager()->getChampSecurite( m_dataAgv->getIdent(),
                                                        DataChampSecurite::type_champ_secu_t( typeChampSecu ),
                                                        m_vitesse,
                                                        geomChampSecuTmp) )
            {
                geomChampSecuTmp = GeomChampSecurite();
            }

            newGeomChampSecu.m_longueurAv = max(newGeomChampSecu.m_longueurAv, geomChampSecuTmp.m_longueurAv);
            newGeomChampSecu.m_longueurAr = max(newGeomChampSecu.m_longueurAr, geomChampSecuTmp.m_longueurAr);
            newGeomChampSecu.m_debordAv = max(newGeomChampSecu.m_debordAv, geomChampSecuTmp.m_debordAv);
            newGeomChampSecu.m_debordAr = max(newGeomChampSecu.m_debordAr, geomChampSecuTmp.m_debordAr);
        }

        // on doit avoir au moins le champ de secu min defini plus la distance de secu
        newGeomChampSecu.m_longueurAv = max( newGeomChampSecu.m_longueurAv, m_champSecuMini.m_longueurAv);
        newGeomChampSecu.m_longueurAv += distSecu;

        newGeomChampSecu.m_longueurAr = max( newGeomChampSecu.m_longueurAr, m_champSecuMini.m_longueurAr);
        newGeomChampSecu.m_longueurAr += distSecu;

        newGeomChampSecu.m_debordAv = max( newGeomChampSecu.m_debordAv, m_champSecuMini.m_debordAv);
        newGeomChampSecu.m_debordAv += distSecu;

        newGeomChampSecu.m_debordAr = max( newGeomChampSecu.m_debordAr, m_champSecuMini.m_debordAr);
        newGeomChampSecu.m_debordAr += distSecu;

        // recalcul du polygone avec champ de securite
        debordMax = max(newGeomChampSecu.m_debordAv, newGeomChampSecu.m_debordAr);
        const double dav = m_dataAgv->getXAvGabaritAgv() + newGeomChampSecu.m_longueurAv;
        const double dar = m_dataAgv->getXArGabaritAgv() - newGeomChampSecu.m_longueurAr;
        const double dg = m_dataAgv->getYGaucheGabaritAgv() + debordMax;
        const double dd = m_dataAgv->getYDroiteGabaritAgv() - debordMax;

        const double halfWidth = (dav-dar)/2;
        const double halfHeigth = (dg-dd)/2;
        const double centreX = m_pointTube->getX() + cos(m_pointTube->getTheta()) * (dav - halfWidth) - sin(m_pointTube->getTheta()) * (dg - halfHeigth);
        const double centreY = m_pointTube->getY() + sin(m_pointTube->getTheta()) * (dav - halfWidth) + cos(m_pointTube->getTheta()) * (dg - halfHeigth);

        const OBB2D rectChampSecu(centreX, centreY, halfWidth, halfHeigth, m_pointTube->getTheta());

        m_pointTube->changeVitesse( m_vitesse,
                                m_vitesseCurrentComputation,
                                &rectChampSecu );
        res = STATUS_OK;
    }

    return res;
}


void
AlgoChangeVitessePointTube::setValues( DataAgv* dataAgv,
                                  int16_t vitesse,
                                  int16_t vitesseCurrentComputation,
                                  bool checkZoneRalenti,
                                  PointTube *pointTube,
                                  const GeomChampSecurite& champSecuMini)
{
    m_dataAgv = dataAgv;
    m_vitesse = vitesse;
    m_vitesseCurrentComputation = vitesseCurrentComputation;
    m_checkZoneRalenti = checkZoneRalenti;
    m_pointTube = pointTube;
    m_champSecuMini = champSecuMini;
}


/* Algo de recherche d'interaction entre un AGV sur un point tub et un troncon
 * la valeur de retour se trouve dans m_isInteraction
 * @return STATUS_ERROR si troncon introuvable ou dataagv null, STATUS_OK sinon
 */
status_t
AlgoIsInteractionAgvTrc::run()
{
    DataPointConnexion *pc;
    double theta;
    CmpTroncon* trc;
    PointTube pointTubeTrc, *ptTubeTmp;
    Tube tubeCible(getCmpDataManager());
    listPointTube_t listTmp, listPointTrc;
    AlgoCalculPointTube algoCalculPointTube(NULL, 0, 0, 0, true, 0, false, false, NULL, GeomChampSecurite(), getCmpDataManager());
    double distCarre;
    GeomChampSecurite champSecuMini;

    //initialisations
    *m_isInteraction = false;
    trc = getCmpDataManager()->getCmpTroncon(m_idTrc);
    if(trc==NULL || m_dataAgv==NULL)
    {
        return STATUS_ERROR;
    }

    getCmpDataManager()->getChampSecurite(m_dataAgv->getIdent(),
                                          0,
                                          champSecuMini);

    //discrimination
    pc = trc->getDataPointConnexionDebut();
    theta= (m_dataAgv->getNbTourelles()>1) ? pc->getGamma() + pc->getAlpha() : pc->getGamma();

    algoCalculPointTube.setValues( m_dataAgv,
                                   pc->getX(),
                                   pc->getY(),
                                   theta,
                                   true,
                                   0,
                                   m_checkZoneRalenti,
                                   false, // point tube reel (pas theorique)
                                   &pointTubeTrc,
                                   champSecuMini);
    algoCalculPointTube.run();

    tubeCible.paramAgv(m_dataAgv->getNumAgv());

    if(PointTube::calculerDistance(m_pointTube,  &pointTubeTrc) < m_pointTube->getRayonAgvChampSecu() + pointTubeTrc.getRayonAgvChampSecu() + trc->getLength())
    {
        //on calculer la liste des pointsTube du troncon
        AlgoListPointTrc algoListPointTrc(m_idTrc, m_dataAgv->getIdent(), &listTmp, getCmpDataManager());
        algoListPointTrc.run();

        //on recopie cette liste pour avoir les polygones des pointTube
        for(listPointTube_t::iterator it=listTmp.begin() ; it!=listTmp.end(); ++it)
        {
            ptTubeTmp = new PointTube();
            algoCalculPointTube.setValues( m_dataAgv,
                                           (*it)->getX(), (*it)->getY(), (*it)->getTheta(),
                                           (*it)->getMarcheAv(),
                                           0,
                                           m_checkZoneRalenti,
                                           true, // point tube theorique
                                           ptTubeTmp,
                                           champSecuMini);
            algoCalculPointTube.run();
            listPointTrc.push_back(ptTubeTmp);
        }

        //on cherche s'il y a interaction
        AlgoIsIntersectionPointTube algoIsIntersectionPointTube(NULL,
                                                                NULL,
                                                                NULL, NULL,
                                                                getCmpDataManager());
        for(listPointTube_t::iterator it=listPointTrc.begin() ;
            it!=listPointTrc.end() && !(*m_isInteraction) ;
            ++it)
        {
            algoIsIntersectionPointTube.setValues( m_pointTube,
                                                   *it,
                                                   m_isInteraction,
                                                   &distCarre);
            algoIsIntersectionPointTube.run();
        }

        for(listPointTube_t::iterator it=listPointTrc.begin(); it!=listPointTrc.end(); ++it)
        {
           delete *it;
        }
    }

    return STATUS_OK;
}

void
AlgoIsInteractionAgvTrc::setValues(ident_t idTrc, PointTube *pointTube,
                                   DataAgv* dataAgv,
                                   bool checkZoneRalenti,
                                   bool *isInteraction)
{
    m_idTrc = idTrc;
    m_pointTube = pointTube;
    m_dataAgv = dataAgv;
    m_checkZoneRalenti = checkZoneRalenti;
    m_isInteraction = isInteraction;
}

