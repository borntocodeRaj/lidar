/* BA Systemes
 * 
 * Projet: 0000Z - SPEED 
 * Composants: C_CIRCUIT_LASER-01
 * 
 * __IDCVS__
 */

// ----------------------------------------------------------------------------

#ifndef __POINTCIRCUIT_HH__
#define __POINTCIRCUIT_HH__

// ------ DI - directives d'inclusion -----------------------------------------

#include "track/lib/circuit.h"
#include "track/lib/data_point_connexion.h"
#include "track/lib/data_point_support.h"

// ------ CP - constantes publiques -------------------------------------------

// ------ MP - macros publiques -----------------------------------------------

// ------ TP - types et structures publics ------------------------------------

// ------ DC - declaration des classes ----------------------------------------

/** Implemente la classe PointCircuit.
 *  Un pointCircuit est un element circuit (ex. de connexion, de support...).
 */
class PointCircuit : public Circuit
{
  ISA(PointCircuit, Circuit);

public:

  // un type sympa...
  enum typePointCircuit_t {
    CONNEXION,
    SUPPORT,
    AUTRE
  };

  // pour comparer les coordonnees de 2 points circuit 
  enum pointCircuitComp_t {
    DIFF,
    IDEM
  };
  PointCircuit( ident_t ident = 0,
		typePointCircuit_t type = AUTRE,
		double x = 0, double y = 0, double gamma = 0, double alpha = 0,
		double phiTourelleAv = 0, double phiTourelleAr = 0);
  PointCircuit( DataPointConnexion* dataPointConnexion );
  PointCircuit( DataPointSupport* dataPointSupport );
  PointCircuit( const PointCircuit &pointCircuit );
  void validate();

    virtual ~PointCircuit();

  void display();

  // accesseurs sur les membres du pointCircuit
  ident_t getIdent() { return m_ident; }
  typePointCircuit_t getType() { return m_type; }
  bool isPointConnexion() { return (m_type == CONNEXION ? true : false); }
  double getX() const { return m_x; }
  double getY() const { return m_y; }
  double getGamma() const { return m_gamma; }
  double getAlpha() const { return m_alpha; }
  double getPhiTourelleAv() const { return m_phiTourelleAv; };
  double getPhiTourelleAr() const { return m_phiTourelleAr; };
  void setIdent( ident_t ident ) { m_ident = ident; }
  void setType( typePointCircuit_t type ) { m_type = type; }
  void setX( double x ) { m_x = x; }
  void setY( double y ) { m_y = y; }
  void setGamma( double gamma ) { m_gamma = gamma; }
  void setAlpha( double alpha ) { m_alpha = alpha; }
  void setPhiTourelleAv( double phiAv) { m_phiTourelleAv = phiAv; };
  void setPhiTourelleAr( double phiAr) { m_phiTourelleAr = phiAr; };

  pointCircuitComp_t comparerCoordonnees( PointCircuit* pointCircuit );
  pointCircuitComp_t comparerCoordonneesXY( PointCircuit* pointCircuit );

private:

  /// ident du point - nul si invalide : tous les points n'ont pas un ident.
  ident_t m_ident;

  /// type du point - cf. typePointCircuit_t.
  typePointCircuit_t m_type;

  /// coordonnees du point - en mm et rad.
  double m_x;
  double m_y;
  double m_gamma; // orientation trajectoire
  double m_alpha; // orientation AGV bi-tourelle relatif
  double m_phiTourelleAv; // angle de la tourelle av dans repere du NP
  double m_phiTourelleAr; // angle de la tourelle ar dans repere du NP
  
};
  
// ------ PF - prototypes des fonctions publiques -----------------------------

// ------ VG - variables globales exportees -----------------------------------

// ----------------------------------------------------------------------------

#endif  // __POINTCIRCUIT_HH__

// ------ FIN -----------------------------------------------------------------
