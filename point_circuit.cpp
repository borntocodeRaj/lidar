/* BA Systemes
 *
 * Projet: 0000Z - SPEED
 * Composants: C_CIRCUIT_lASER-01
 *
 * __IDCVS__
 */

// ----------------------------------------------------------------------------

/** @file
 *  Implemente la classe PointCircuit.
 *  Un pointCircuit est un element circuit (ex. de connexion, de support...).
 */

// ------ DI - directives d'inclusion -----------------------------------------

#include <math.h>

#include "track/lib/point_circuit.h"
#include "tools/ba/misc/util.h"

// ------ CP - constantes privées ---------------------------------------------

// ------ MP - macros privees -------------------------------------------------

// ------ TP - types et structures prives -------------------------------------

// ------ VG - variables globales publiques et privees ------------------------

// ------ PF - prototypes des fonctions privees -------------------------------

// ------ CF - code des methodes ----------------------------------------------

/** Constructeur.
 *  @param ident peut etre nul si point sans ident.
 *  @param type -
 *  @param x en mm
 *  @param y en mm
 *  @param gamma en rad dans ]-pi/2,+pi/2]
 */
PointCircuit::PointCircuit( ident_t ident, PointCircuit::typePointCircuit_t type,
                            double x, double y, double gamma, double alpha,
                            double phiTourelleAv, double phiTourelleAr)
  : Circuit(),
    m_ident(ident), m_type(type), m_x(x), m_y(y), m_gamma(gamma), m_alpha(alpha),
	m_phiTourelleAv(phiTourelleAv), m_phiTourelleAr(phiTourelleAr)

{
  validate();
}

/** Un autre constructeur pour les points de connexion.
 *  @param dataPointConnexion une data a recopier dans le point.
 */

PointCircuit::PointCircuit( DataPointConnexion* dataPointConnexion )
  : Circuit()
{
  // recopie des donnees
  m_ident = dataPointConnexion->getIdent();
  m_type  = CONNEXION;
  m_x     = dataPointConnexion->getX();
  m_y     = dataPointConnexion->getY();
  m_gamma = Util::modulo2PI(dataPointConnexion->getGamma());
  m_alpha = Util::modulo2PI(dataPointConnexion->getAlpha());
  m_phiTourelleAv = 0.0;
  m_phiTourelleAr = 0.0;

  validate();
}

/** Un autre constructeur pour les points de support.
 *  @param dataPointSupport une data a recopier dans le point.
 */

PointCircuit::PointCircuit( DataPointSupport* dataPointSupport )
  : Circuit()
{
  // recopie des donnees
  m_ident = dataPointSupport->getIdent();
  m_type  = SUPPORT;
  m_x     = dataPointSupport->getX();
  m_y     = dataPointSupport->getY();
  m_gamma = Util::modulo2PI(dataPointSupport->getGamma());
  m_alpha = Util::modulo2PI(dataPointSupport->getAlpha());
  m_phiTourelleAv = Util::modulo2PI(dataPointSupport->getPhiTourelleAv());
  m_phiTourelleAr = Util::modulo2PI(dataPointSupport->getPhiTourelleAr());

  validate();
}

/** Constructeur par recopie.
 */
PointCircuit::PointCircuit(const PointCircuit &pointCircuit)
  : Circuit()
{
  m_ident       = pointCircuit.m_ident;
  m_type        = pointCircuit.m_type;
  m_x           = pointCircuit.m_x;
  m_y           = pointCircuit.m_y;
  m_gamma       = Util::modulo2PI(pointCircuit.m_gamma);
  m_alpha       = Util::modulo2PI(pointCircuit.m_alpha);
  m_phiTourelleAv = Util::modulo2PI(pointCircuit.m_phiTourelleAv);
  m_phiTourelleAr = Util::modulo2PI(pointCircuit.m_phiTourelleAr);

  validate();
}

void
PointCircuit::validate()
{
  // validations sur les donnees de creation du point
  BA_I( m_type == CONNEXION || m_type == SUPPORT || m_type == AUTRE );
  //BA_I( m_gamma > -M_PI && m_gamma <= M_PI );
  m_gamma = Util::modulo2PI(m_gamma);
  m_alpha = Util::modulo2PI(m_alpha);
  m_phiTourelleAv = Util::modulo2PI(m_phiTourelleAv);
  m_phiTourelleAr = Util::modulo2PI(m_phiTourelleAr);

    // validations sur les donnees de creation du point
  BA_I( m_gamma > -M_PI && m_gamma <= M_PI );
  BA_I( m_alpha > -M_PI && m_alpha <= M_PI );
  BA_I( m_phiTourelleAv > -M_PI && m_phiTourelleAv <= M_PI );
  BA_I( m_phiTourelleAr > -M_PI && m_phiTourelleAr <= M_PI );
}

PointCircuit::~PointCircuit()
{
}

/** Affichage des attributs du point.
 */
void
PointCircuit::display()
{
  clog << "Ident = " << m_ident << " - type = ";
  switch (m_type)
    {
    case PointCircuit::CONNEXION :
      clog << "CONNEXION" << endl;
      break;
    case PointCircuit::SUPPORT :
      clog << "SUPPORT" << endl;
      break;
    case PointCircuit::AUTRE :
      clog << "AUTRE" << endl;
      break;
    default :
      clog << "INVALIDE !" << endl;
    };
  clog << " x : " << m_x << " - y : " << m_y << " - gamma : " <<  m_gamma << " - alpha : " <<  m_alpha <<
		  " m_phiTourelleAv : " << m_phiTourelleAv << " m_phiTourelleAr : " << m_phiTourelleAr << endl;

}

/** Compare les coordonnees avec un autre point circuit.
 *  Precision : 1 mm pour x et y, 0.01 degre pour gamma. On ne teste pas l'alpha
 *  ni l'angle des tourelles
 *  Pas de test sur ident et type du point.
 *  @param pointCircuit -
 *  @return IDEM si coordonnees identiques, DIFF sinon
 */
PointCircuit::pointCircuitComp_t
PointCircuit::comparerCoordonnees( PointCircuit* pointCircuit )
{
  double dX = fabs(m_x - pointCircuit->getX());
  double dY = fabs(m_y - pointCircuit->getY());
  double dGamma = fabs(m_gamma - pointCircuit->getGamma());

  return ( ( dX < 1.0 && dY < 1.0 && dGamma < Util::convertDeg2Rad(0.01) )
           ? IDEM : DIFF );
}

PointCircuit::pointCircuitComp_t
PointCircuit::comparerCoordonneesXY( PointCircuit* pointCircuit )
{
  double dX = fabs(m_x - pointCircuit->getX());
  double dY = fabs(m_y - pointCircuit->getY());

  return ( ( dX < 1.0 && dY < 1.0 )
           ? IDEM : DIFF );
}
// ------ FIN -----------------------------------------------------------------
