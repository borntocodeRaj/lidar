//! � BA Robotic Systems Group
//! This file and its content are the property of BA Robotic Systems Group.
//! Any reproduction or use outside BA Robotic Systems Group is prohibited.
//!
//! @file AlgoGotoXYGamma.cpp
// Author : Rajkrishna Paul

// ------ DI - directives d'inclusion -----------------------------------------

#include "AlgoGotoXYGamma.h"
#include <stack>
#include <sstream> //ne sert que dans les classes derivees
#include "agv/track/lib/algo_cmp.h"
#include "agv/track/lib/list_msg_info.h"
#include "agv/track/lib/algo_notify_point_connexion.h"
#include "agv/track/lib/algo_cmp_transpondeur.h"
#include "agv/track/lib/algo_cmp_troncon.h"
#include "agv/track/lib/algo_data_porte.h"
#include "agv/track/lib/algo_cmp_agv.h"
#include "agv/track/lib/algo_intersections.h"
#include "agv/track/lib/cmp_data_manager.h"
#include "agv/tools/ba/misc/parser.h"
#include "agv/tools/ba/misc/util.h"
#include "agv/track/lib/algo_cmp_zone.h"
#include "agv/track/lib/algo_intersections.h"
#include "agv/tools/ba/misc/util.h"
#include "agv/track/lib/segment_courbe.h"
#include "agv/misc/matrix.h"
#include "agv/tools/ba/misc/parser.h"
#include "agv/tools/ba/misc/util.h"
#include "agv/tools/ba/misc/misc.h"
#include "agv/track/lib/algo_cmp_zone.h"
#include "agv/tools/ba/misc/util.h"
#include "agv/track/lib/segment_courbe.h"
#include "agv/tools/ba/misc/parser.h"
#include "agv/tools/ba/misc/util.h"
#include "agv/tools/ba/misc/misc.h"
#include "agv/track/lib/algo_cmp_zone.h"
#include "agv/tools/ba/misc/util.h"
#include "agv/track/lib/segment_courbe.h"



class AlgoGotoXYGamma;
vector<pointGeom_t> polygon(pointGeom_t A,pointGeom_t B,
                            pointGeom_t C,pointGeom_t D, pointGeom_t E, pointGeom_t F)

                 {
    vector<pointGeom_t> polygon;
    polygon.push_back(A);
    polygon.push_back(B);
    polygon.push_back(C);
    polygon.push_back(D);
    polygon.push_back(E);
    polygon.push_back(F);

    return polygon;

                 }

void AlgoGotoXYGamma::Init()
{
    m_workingArea = polygon(AA1, AAx1, AAx, A1, Ax, A1x);

    CmpAgv * myAgv = m_cmpMng->getCmpAgv(1);
    double xAvGabarit = myAgv->getDataAgv()->getXAvGabaritAgv();
    double xArGabarit = myAgv->getDataAgv()->getXArGabaritAgv();
    double ygabaritGauche = myAgv->getDataAgv()->getYGaucheGabaritAgv();
    double ygabaritDroite = myAgv->getDataAgv()->getYDroiteGabaritAgv();
    if(myAgv != NULL)
    {
        m_agv_gabaritxv = xAvGabarit ;
        m_agv_gabaritxr = xArGabarit ;
        m_agv_gabarityGauche = ygabaritGauche;
        m_agv_gabarityDroite = ygabaritDroite;
    }
}

AlgoGotoXYGamma::AlgoGotoXYGamma(CmpDataManager* cmpDataManager):
                  m_current_troncon_Id(0),
                  m_cmpMng(cmpDataManager),
                  m_agv_gabaritxv(0.0),
                  m_agv_gabaritxr(0.0),
                  m_agv_gabarityGauche(0.0),
                  m_agv_gabarityDroite(0.0)
{
    Init();
}


void AlgoGotoXYGamma::setWorkingArea(vector<pointGeom_t>  _polygon)
{
    _polygon = m_workingArea;


}
bool finalvite,op;

bool AlgoGotoXYGamma::isAllowed(PointCircuit * pt) const
{
    vector<pointGeom_t>* res = new vector<pointGeom_t>();

    double x = pt->getX();
    double y = pt->getY();
    double gamma = pt->getGamma();

    vector<pointGeom_t> agvpolygone;
    //DataAgv *agv = m_cmpMng->getCmpAgv(1)->getDataAgv();
    double xAv = m_agv_gabaritxv;// agv->getXAvGabaritAgv();
    double xAr = m_agv_gabaritxr;
    double yGauche = m_agv_gabarityGauche;
    double yDroite = m_agv_gabarityDroite;


    AlgoCalculPolygone algoPolygone(&agvpolygone,x,y,gamma,xAv,xAr,yGauche,yDroite,m_cmpMng);
    algoPolygone.run();

    AlgoIntersectionPairePolygone algoIntersectionPairePolygone(&m_workingArea,&agvpolygone,
                                                                res,finalvite, m_cmpMng);
   algoIntersectionPairePolygone.run();


    int no_Of_Points_Of_Intersection = res->size();

    if(no_Of_Points_Of_Intersection == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool AlgoGotoXYGamma::isAllowed(CmpTroncon * cmpTrc) const
{

    double length = cmpTrc->getLength();
    ServicesInfoTrajectoire serviceInfo;
    PointCircuit * myPt = new PointCircuit(0, PointCircuit::AUTRE, 0,0,0 );


    for (double s = 0; s <= length; s+=10)
    {
        status_t res = serviceInfo.calculerPointSurTroncon(cmpTrc, cmpTrc->getDataAgv(), s, myPt);
        if (res == STATUS_OK)
        {
            if(!isAllowed(myPt))
            {
                return false;
            }

            return true;
        }

    }

    return false;
}

bool AlgoGotoXYGamma::gotoXYGamma(double _x, double _y, double _gamma, ident_t m_current_troncon_Id)
{

    CmpTroncon* cmpTrc = m_cmpMng->getCmpTroncon(m_current_troncon_Id);
    LogInfo <<"The value of cmpTrc  "<< (int) cmpTrc <<std::endl;

    if(cmpTrc != NULL)
    {
        DataPointConnexion * pcFin = cmpTrc->getDataPointConnexionFin();
        double X = pcFin->getX();
        double Y = pcFin->getY();
        double Gamma= pcFin->getGamma();
        PointCircuit * myPt = new PointCircuit(0, PointCircuit::AUTRE, _x,_y,_gamma);
        AlgoNotifyCoordOfPtCon * algoNotify = new AlgoNotifyCoordOfPtCon(pcFin->getIdent(),
                                                                         false,
                                                                         _x,
                                                                         _y,
                                                                         _gamma,
                                                                         m_cmpMng,
                                                                         0);

        LogInfo <<"RPA case 0 "<< algoNotify <<std::endl;
        status_t resNotify = algoNotify->run();
        if(resNotify == STATUS_OK)
        {
            LogInfo <<"RPA case 01 "<< algoNotify <<std::endl;
            if(!isAllowed(myPt))
            {
                AlgoNotifyCoordOfPtCon * algoNotify = new AlgoNotifyCoordOfPtCon(pcFin->getIdent(),
                                                                                 false,
                                                                                 X,
                                                                                 Y,
                                                                                 Gamma,
                                                                                 m_cmpMng,
                                                                                 0);
                algoNotify->run();
                LogInfo <<"RPA case 1  "<< algoNotify <<std::endl;
                return false;

            }

            else
            {
                LogInfo <<"RPA case 2  "<< algoNotify <<std::endl;
                return true;
            }
        }
        LogInfo <<"RPA case 3  "<< algoNotify <<std::endl;

    }
    LogInfo <<" Default value when false " <<std::endl;
    return false;
}

//ident_t m_current_troncon_Id
bool AlgoGotoXYGamma::fitsIn(ident_t m_current_troncon_Id)
{
    bool res = true;
    CmpTroncon* cmpTrc = m_cmpMng->getCmpTroncon(m_current_troncon_Id);
    DataPointConnexion * ptDebut = cmpTrc->getDataPointConnexionDebut();
    DataPointConnexion * ptFin = cmpTrc->getDataPointConnexionFin();
    double gamma_d = ptDebut->getGamma();
    double gamma_f = ptFin ->getGamma();
    if(cmpTrc != NULL  )
    {
        if (gamma_d == gamma_f)

        {
            res = isAllowed(cmpTrc);
            return res;
        }

        if (gamma_f - gamma_d < -90 && gamma_f - gamma_d > -154)
        {

            return false;

        }

        if (gamma_f - gamma_d > -90)

        {
            res = isAllowed(cmpTrc);
            return res;
        }

        if (gamma_f - gamma_d < -156)
        {
            res = isAllowed(cmpTrc);
            return res;
        }


    }
    else
    {

        return false;
    }

    return res;

}
