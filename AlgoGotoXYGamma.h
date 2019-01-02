//! � BA Robotic Systems Group
//! This file and its content are the property of BA Robotic Systems Group.
//! Any reproduction or use outside BA Robotic Systems Group is prohibited.
//!
//! @file AlgoGotoXYGamma.h
// Author : Rajkrishna Paul

#ifndef TRACK_LIB_ALGOGOTOXYGAMMA_H_
#define TRACK_LIB_ALGOGOTOXYGAMMA_H_
#include <string>
#include "agv/track/lib/cmp_data_manager.h"
#include "agv/track/lib/algo_cmp_troncon.h"
#include "src/log/logger.hpp"
#include "agv/os_services/src/log/StreamLogger.hpp"
const pointGeom_t AAx1(-1000,0);
const pointGeom_t AAx(-1000,1000);
const pointGeom_t AAa(0,-2000);
const pointGeom_t AA1(-1000,-1000);
const pointGeom_t AA2(-1000,-2000);
const pointGeom_t AA3(-1000, -3000);
const pointGeom_t AA4(-1000, -4000);
const pointGeom_t AA5(-1000, -5000);
const pointGeom_t AA6(-1000, -6000);
const pointGeom_t AA7(-1000, -7000);
const pointGeom_t AA8(-1000, -8000);
const pointGeom_t AA9(-1000, -9000);
const pointGeom_t BB0(-2000, 0);
const pointGeom_t BB1(-2000, -1000);
const pointGeom_t BB2(-2000, -2000);
const pointGeom_t BB3(-2000, -3000);
const pointGeom_t BB4(-2000, -4000);
const pointGeom_t BB5(-2000, -5000);
const pointGeom_t BB(-2000, -6000);
const pointGeom_t BB7(-2000, -7000);
const pointGeom_t BB8(-2000, -8000);
const pointGeom_t BB9(-2000, -9000);
const pointGeom_t CC0(-3000, 0);
const pointGeom_t CC1(-3000, -1000);
const pointGeom_t CC2(-3000, -2000);
const pointGeom_t CC3(-3000, -3000);
const pointGeom_t CC4(-3000, -4000);
const pointGeom_t CC5(-3000, -5000);
const pointGeom_t CC6(-3000, -6000);
const pointGeom_t CC7(-3000, -7000);
const pointGeom_t CC8(-3000, -8000);
const pointGeom_t CC9(-3000, -9000);
const pointGeom_t DD0(-4000, 0);
const pointGeom_t DD1(-4000, -1000);
const pointGeom_t DD2(-4000, -2000);
const pointGeom_t DD3(-4000, -3000);
const pointGeom_t DD4(-4000, -4000);
const pointGeom_t DD5(-4000, -5000);
const pointGeom_t DD6(-4000, -6000);
const pointGeom_t DD7(-4000, -7000);
const pointGeom_t DD8(-4000, -8000);
const pointGeom_t DD9(-4000, -9000);
const pointGeom_t EE0(-5000, 0);
const pointGeom_t EE1(-5000, -1000);
const pointGeom_t EE2(-5000, -2000);
const pointGeom_t EE3(-5000, -3000);
const pointGeom_t EE4(-5000, -4000);
const pointGeom_t EE5(-5000, -5000);
const pointGeom_t EE6(-5000, -6000);
const pointGeom_t EE7(-5000, -7000);
const pointGeom_t EE8(-5000, -8000);
const pointGeom_t EE9(-5000, -9000);
const pointGeom_t FF0(-6000, 0);
const pointGeom_t FF1(-6000, -1000);
const pointGeom_t FF2(-6000, -2000);
const pointGeom_t FF3(-6000, -3000);
const pointGeom_t FF4(-6000, -4000);
const pointGeom_t FF5(-6000, -5000);
const pointGeom_t FF6(-6000, -6000);
const pointGeom_t FF7(-6000, -7000);
const pointGeom_t FF8(-6000, -8000);
const pointGeom_t FF9(-6000, -9000);
const pointGeom_t GG0(-7000, 0);
const pointGeom_t GG1(-7000, -1000);
const pointGeom_t GG2(-7000, -2000);
const pointGeom_t GG3(-7000, -3000);
const pointGeom_t GG4(-7000, -4000);
const pointGeom_t GG5(-7000, -5000);
const pointGeom_t GG6(-7000, -6000);
const pointGeom_t GG7(-7000, -7000);
const pointGeom_t GG8(-7000, -8000);
const pointGeom_t GG9(-7000, -9000);
const pointGeom_t HH0(-8000, 0);
const pointGeom_t HH1(-8000, -1000);
const pointGeom_t HH2(-8000, -2000);
const pointGeom_t HH3(-8000, -3000);
const pointGeom_t HH4(-8000, -4000);
const pointGeom_t HH5(-8000, -5000);
const pointGeom_t HH6(-8000, -6000);
const pointGeom_t HH7(-8000, -7000);
const pointGeom_t HH8(-8000, -8000);
const pointGeom_t HH9(-8000, -9000);
const pointGeom_t II0(-9000, -0);
const pointGeom_t II1(-9000, -1000);
const pointGeom_t II2(-9000, -2000);
const pointGeom_t II3(-9000, -3000);
const pointGeom_t II4(-9000, -4000);
const pointGeom_t II5(-9000, -5000);
const pointGeom_t II6(-9000, -6000);
const pointGeom_t II7(-9000, -7000);
const pointGeom_t II8(-9000, -8000);
const pointGeom_t II9(-9000, -9000);
const pointGeom_t JJ0(-10000, 0);
const pointGeom_t JJ1(-10000, -1000);
const pointGeom_t JJ2(-10000, -2000);
const pointGeom_t JJ3(-10000, -3000);
const pointGeom_t JJ4(-10000, -4000);
const pointGeom_t JJ5(-10000, -5000);
const pointGeom_t JJ6(-10000, -6000);
const pointGeom_t JJ7(-10000, -7000);
const pointGeom_t JJ8(-10000, -8000);
const pointGeom_t JJ9(-10000, -9000);
const pointGeom_t KK1(-11000, -1000);
const pointGeom_t KK2(-11000, -2000);
const pointGeom_t KK3(-11000, -3000);
const pointGeom_t KK4(-11000, -4000);
const pointGeom_t KK5(-11000, -5000);
const pointGeom_t KK6(-11000, -6000);
const pointGeom_t KK7(-11000, -7000);
const pointGeom_t KK8(-11000, -8000);
const pointGeom_t KK9(-11000, -9000);
const pointGeom_t LL1(-12000, -1000);
const pointGeom_t LL2(-12000, -2000);
const pointGeom_t LL3(-12000, -3000);
const pointGeom_t LL4(-12000, -4000);
const pointGeom_t LL5(-12000, -5000);
const pointGeom_t LL6(-12000, -6000);
const pointGeom_t LL7(-12000, -7000);
const pointGeom_t LL8(-12000, -8000);
const pointGeom_t LL9(-12000, -9000);
const pointGeom_t MM0(-13000, 0);
const pointGeom_t MM1(-13000, -1000);
const pointGeom_t MM2(-13000, -2000);
const pointGeom_t MM3(-13000, -3000);
const pointGeom_t MM4(-13000, -4000);
const pointGeom_t MM5(-13000, -5000);
const pointGeom_t MM6(-13000, -6000);
const pointGeom_t MM7(-13000, -7000);
const pointGeom_t MM8(-13000, -8000);
const pointGeom_t MM9(-13000, -9000);
const pointGeom_t NN1(-14000, -1000);
const pointGeom_t NN2(-14000, -2000);
const pointGeom_t NN3(-14000, -3000);
const pointGeom_t NN4(-14000, -4000);
const pointGeom_t NN5(-14000, -5000);
const pointGeom_t NN6(-14000, -6000);
const pointGeom_t NN7(-14000, -7000);
const pointGeom_t NN8(-14000, -8000);
const pointGeom_t NN9(-14000, -9000);
const pointGeom_t OO1(-15000, -1000);
const pointGeom_t OO2(-15000, -2000);
const pointGeom_t OO3(-15000, -3000);
const pointGeom_t OO4(-15000, -4000);
const pointGeom_t OO5(-15000, -5000);
const pointGeom_t OO6(-15000, 6000);
const pointGeom_t OO7(-15000, -7000);
const pointGeom_t OO8(-15000, -8000);
const pointGeom_t O09(-15000, -9000);
const pointGeom_t PP1(-16000, -1000);
const pointGeom_t PP2(-16000, -2000);
const pointGeom_t PP3(-16000, -3000);
const pointGeom_t PP4(-16000, -4000);
const pointGeom_t PP5(-16000, -5000);
const pointGeom_t PP6(-16000, -6000);
const pointGeom_t PP7(-16000, -7000);
const pointGeom_t PP8(-16000, -8000);
const pointGeom_t PP9(-16000, -9000);
const pointGeom_t QQ1(-17000, -1000);
const pointGeom_t QQ2(-17000, -2000);
const pointGeom_t QQ3(-17000, -3000);
const pointGeom_t QQ4(-17000, -4000);
const pointGeom_t QQ5(-17000, -5000);
const pointGeom_t QQ6(-17000, -6000);
const pointGeom_t QQ7(-17000, -7000);
const pointGeom_t QQ8(-17000, -8000);
const pointGeom_t QQ9(-17000, -9000);
const pointGeom_t RR1(-18000, -1000);
const pointGeom_t RR2(-18000, -2000);
const pointGeom_t RR3(-18000, -3000);
const pointGeom_t RR4(-18000, -4000);
const pointGeom_t RR5(-18000, -5000);
const pointGeom_t RR6(-18000, -6000);
const pointGeom_t RR7(-18000, -7000);
const pointGeom_t RR8(-18000, -8000);
const pointGeom_t RR9(-18000, -9000);
const pointGeom_t Ax(1000,0);
const pointGeom_t Aa(0,2000);
const pointGeom_t A1(1000,1000);
const pointGeom_t A1x(1000,-1000);
const pointGeom_t A2(1000,2000);
const pointGeom_t A3(1000, 3000);
const pointGeom_t A4(1000, 4000);
const pointGeom_t A5(1000, 5000);
const pointGeom_t A6(1000, 6000);
const pointGeom_t A7(1000, 7000);
const pointGeom_t A8(1000, 8000);
const pointGeom_t A9(1000, 9000);
const pointGeom_t B0(2000, 0);
const pointGeom_t B1(2000, 1000);
const pointGeom_t B2(2000, 2000);
const pointGeom_t B3(2000, 3000);
const pointGeom_t B4(2000, 4000);
const pointGeom_t B5(2000, 5000);
const pointGeom_t B6(2000, 6000);
const pointGeom_t B7(2000, 7000);
const pointGeom_t B8(2000, 8000);
const pointGeom_t B9(2000, 9000);
const pointGeom_t C0(3000, 0);
const pointGeom_t C1(3000, 1000);
const pointGeom_t C2(3000, 2000);
const pointGeom_t C3(3000, 3000);
const pointGeom_t C4(3000, 4000);
const pointGeom_t C5(3000, 5000);
const pointGeom_t C6(3000, 6000);
const pointGeom_t C7(3000, 7000);
const pointGeom_t C8(3000, 8000);
const pointGeom_t C9(3000, 9000);
const pointGeom_t D0(4000, 0);
const pointGeom_t D1(4000, 1000);
const pointGeom_t D2(4000, 2000);
const pointGeom_t D3(4000, 3000);
const pointGeom_t D4(4000, 4000);
const pointGeom_t D5(4000, 5000);
const pointGeom_t D6(4000, 6000);
const pointGeom_t D7(4000, 7000);
const pointGeom_t D8(4000, 8000);
const pointGeom_t D9(4000, 9000);
const pointGeom_t E0(5000, 0);
const pointGeom_t E1(5000, 1000);
const pointGeom_t E2(5000, 2000);
const pointGeom_t E3(5000, 3000);
const pointGeom_t E4(5000, 4000);
const pointGeom_t E5(5000, 5000);
const pointGeom_t E6(-5000, 6000);
const pointGeom_t E7(5000, 7000);
const pointGeom_t E8(5000, 8000);
const pointGeom_t E9(5000, 9000);
const pointGeom_t F0(6000, 0);
const pointGeom_t F1(6000, 1000);
const pointGeom_t F2(6000, 2000);
const pointGeom_t F3(6000, 3000);
const pointGeom_t F4(6000, 4000);
const pointGeom_t F5(6000, 5000);
const pointGeom_t F6(6000, 6000);
const pointGeom_t F7(6000, -7000);
const pointGeom_t F8(6000, 8000);
const pointGeom_t F9(6000, 9000);
const pointGeom_t G0(7000, 0);
const pointGeom_t G1(7000, 1000);
const pointGeom_t G2(7000, 2000);
const pointGeom_t G3(7000, -3000);
const pointGeom_t G4(7000, 4000);
const pointGeom_t G5(7000, 5000);
const pointGeom_t G6(7000, 6000);
const pointGeom_t G7(7000, 7000);
const pointGeom_t G8(7000, 8000);
const pointGeom_t G9(7000, 9000);
const pointGeom_t H0(8000, 0);
const pointGeom_t H1(8000, 1000);
const pointGeom_t H2(8000, 2000);
const pointGeom_t H3(8000, 3000);
const pointGeom_t H4(8000, 4000);
const pointGeom_t H5(8000, 5000);
const pointGeom_t H6(8000, 6000);
const pointGeom_t H7(8000, 7000);
const pointGeom_t H8(8000, 8000);
const pointGeom_t H9(8000, 9000);
const pointGeom_t I0(9000, 0);
const pointGeom_t I1(9000, 1000);
const pointGeom_t I2(9000, 2000);
const pointGeom_t I3(9000, 3000);
const pointGeom_t I4(9000, 4000);
const pointGeom_t I5(9000, 5000);
const pointGeom_t I6(9000, 6000);
const pointGeom_t I7(9000, 7000);
const pointGeom_t I8(9000, 8000);
const pointGeom_t I9(9000, 9000);
const pointGeom_t J0(10000, 0);
const pointGeom_t J1(10000, 1000);
const pointGeom_t J2(10000, 2000);
const pointGeom_t J3(10000, 3000);
const pointGeom_t J4(10000, 4000);
const pointGeom_t J5(10000, 5000);
const pointGeom_t J6(10000, 6000);
const pointGeom_t J7(10000, 7000);
const pointGeom_t J8(10000, 8000);
const pointGeom_t J9(10000, 9000);
const pointGeom_t K1(11000, 1000);
const pointGeom_t K2(11000, 2000);
const pointGeom_t K3(11000, 3000);
const pointGeom_t K4(11000, 4000);
const pointGeom_t K5(11000, 5000);
const pointGeom_t K6(11000, 6000);
const pointGeom_t K7(11000, 7000);
const pointGeom_t K8(11000, 8000);
const pointGeom_t K9(11000, 9000);
const pointGeom_t L1(12000, 1000);
const pointGeom_t L2(12000, 2000);
const pointGeom_t L3(12000, 3000);
const pointGeom_t L4(12000, 4000);
const pointGeom_t L5(12000, 5000);
const pointGeom_t L6(12000, 6000);
const pointGeom_t L7(12000, 7000);
const pointGeom_t L8(12000, 8000);
const pointGeom_t L9(12000, 9000);
const pointGeom_t M0(13000, 0);
const pointGeom_t M1(13000, 1000);
const pointGeom_t M2(13000, 2000);
const pointGeom_t M3(13000, 3000);
const pointGeom_t M4(13000, 4000);
const pointGeom_t M5(13000, 5000);
const pointGeom_t M6(13000, 6000);
const pointGeom_t M7(13000, 7000);
const pointGeom_t M8(13000, 8000);
const pointGeom_t M9(13000, 9000);
const pointGeom_t N1(14000, 1000);
const pointGeom_t N2(14000, 2000);
const pointGeom_t N3(14000, 3000);
const pointGeom_t N4(14000, 4000);
const pointGeom_t N5(14000, 5000);
const pointGeom_t N6(14000, 6000);
const pointGeom_t N7(14000, 7000);
const pointGeom_t N8(14000, 8000);
const pointGeom_t N9(14000, 9000);
const pointGeom_t O1(15000, 1000);
const pointGeom_t O2(15000, 2000);
const pointGeom_t O3(15000, 3000);
const pointGeom_t O4(15000, 4000);
const pointGeom_t O5(15000, 5000);
const pointGeom_t O6(15000, 6000);
const pointGeom_t O7(15000, 7000);
const pointGeom_t O8(15000, 8000);
const pointGeom_t O9(15000, -9000);
const pointGeom_t P1(16000, 1000);
const pointGeom_t P2(16000, 2000);
const pointGeom_t P3(16000, 3000);
const pointGeom_t P4(16000, 4000);
const pointGeom_t P5(16000, 5000);
const pointGeom_t P6(16000, 6000);
const pointGeom_t P7(16000, 7000);
const pointGeom_t P8(16000, 8000);
const pointGeom_t P9(16000, 9000);
const pointGeom_t Q1(17000, 1000);
const pointGeom_t Q2(17000, 2000);
const pointGeom_t Q3(17000, 3000);
const pointGeom_t Q4(17000, 4000);
const pointGeom_t Q5(17000, 5000);
const pointGeom_t Q6(17000, 6000);
const pointGeom_t Q7(17000, 7000);
const pointGeom_t Q8(17000, 8000);
const pointGeom_t Q9(17000, 9000);
const pointGeom_t R1(18000, 1000);
const pointGeom_t R2(18000, 2000);
const pointGeom_t R3(18000, -3000);
const pointGeom_t R4(18000, 4000);
const pointGeom_t R5(18000, 5000);
const pointGeom_t R6(18000, 6000);
const pointGeom_t R7(18000, 7000);
const pointGeom_t R8(18000, 8000);
const pointGeom_t R9(18000, 9000);
const pointGeom_t Origin(3000,2000);


class AlgoGotoXYGamma
{
    public:
        AlgoGotoXYGamma(CmpDataManager* cmpDataManager);
        virtual ~AlgoGotoXYGamma() {}
        void setCurrentTronconId(ident_t _id) { m_current_troncon_Id = _id; }
        void setAGVxAvGabarit(double xAv) {m_agv_gabaritxv = xAv;}
        void setAGVxArGabarit(double xAr) {m_agv_gabaritxr = xAr;}
        void setAGVyGaucheGabarit(double yGauche) {m_agv_gabarityGauche = yGauche;}
        void setAGVyDroiteGabarit(double yDroite) {m_agv_gabarityDroite = yDroite;}
        void setWorkingArea(vector<pointGeom_t>  _polygon);
        bool isAllowed(PointCircuit * _pt) const;
        bool isAllowed(CmpTroncon * _cmpTrc) const;
        bool gotoXYGamma(double _x, double _y, double _gamma,ident_t m_current_troncon_Id);
        bool fitsIn(ident_t m_current_troncon_Id);

    private:
        ident_t m_current_troncon_Id;
        vector<pointGeom_t> m_workingArea;
        CmpDataManager * m_cmpMng;
        double m_agv_gabaritxv,m_agv_gabaritxr,m_agv_gabarityGauche,m_agv_gabarityDroite;
        void Init();
    protected:
        StreamLogger LogInfo;
        StreamLogger LogError;
        StreamLogger LogDebug;

};



#endif /* TRACK_LIB_ALGOGOTOXYGAMMA_H_ */
