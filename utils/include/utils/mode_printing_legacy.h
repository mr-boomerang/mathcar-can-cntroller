// This is not useful anymore, but there just for reference.
#if 0
#ifndef DEFS_H_BKFT10EO
#define DEFS_H_BKFT10EO

#include <iostream>
#include <string.h>

// Colors for printing
#define RST "\E[0m"
#define DGR "\E[1;30m"
#define RED "\E[31m"
#define GRN "\E[32m"
#define YLW "\E[33m"
#define BLU "\E[34m"
#define PUR "\E[35m"
#define CYN "\E[36m"
#define WHT "\E[37m"

//Print in various forms
// Predefined Values for printing
#define PRINT_ERROR std::cout << RED "Error: " RST
#define PRINT_WARN std::cout << YLW "Warning: " RST
#define PRINT_SUCCESS std::cout << GRN "Success: " RST

inline const char* baseFileName(const char* fullname) {
  const char* name = strrchr(fullname, '/');
  return (name==NULL)?fullname:(name+1);
}

#define PRINT_LOC                                                              \
  std::cout << baseFileName(__FILE__) << ":" << __func__ << ":" << __LINE__    \
            << ": "
#define PRINT_LOC_DGR                                                          \
  DGR << baseFileName(__FILE__) << ":" << __func__ << ":" << __LINE__          \
      << ": " RST
#define PRINT_LOC_RED                                                          \
  RED << baseFileName(__FILE__) << ":" << __func__ << ":" << __LINE__          \
      << ": " RST
#define PRINT_LOC_GRN                                                          \
  GRN << baseFileName(__FILE__) << ":" << __func__ << ":" << __LINE__          \
      << ": " RST
#define PRINT_LOC_YLW                                                          \
  YLW << baseFileName(__FILE__) << ":" << __func__ << ":" << __LINE__          \
      << ": " RST
#define PRINT_LOC_BLU                                                          \
  BLU << baseFileName(__FILE__) << ":" << __func__ << ":" << __LINE__          \
      << ": " RST
#define PRINT_LOC_PUR                                                          \
  PUR << baseFileName(__FILE__) << ":" << __func__ << ":" << __LINE__          \
      << ": " RST
#define PRINT_LOC_CYN                                                          \
  CYN << baseFileName(__FILE__) << ":" << __func__ << ":" << __LINE__          \
      << ": " RST

#define PRINT_LOC_D PRINT_LOC_DGR
#define PRINT_LOC_R PRINT_LOC_RED
#define PRINT_LOC_G PRINT_LOC_GRN
#define PRINT_LOC_Y PRINT_LOC_YLW
#define PRINT_LOC_B PRINT_LOC_BLU
#define PRINT_LOC_P PRINT_LOC_PUR
#define PRINT_LOC_C PRINT_LOC_CYN

#define PRINT_ENDL std::cout << std::endl
#define COS << " " <<

#define PDL std::cout << PRINT_LOC_DGR
#define PRL std::cout << PRINT_LOC_RED
#define PGL std::cout << PRINT_LOC_GRN
#define PBL std::cout << PRINT_LOC_BLU
#define PYL std::cout << PRINT_LOC_YLW
#define PPL std::cout << PRINT_LOC_PUR
#define PCL std::cout << PRINT_LOC_CYN

#define PDLE std::cout << PRINT_LOC_DGR << std::endl
#define PRLE std::cout << PRINT_LOC_RED << std::endl
#define PGLE std::cout << PRINT_LOC_GRN << std::endl
#define PBLE std::cout << PRINT_LOC_BLU << std::endl
#define PYLE std::cout << PRINT_LOC_YLW << std::endl
#define PPLE std::cout << PRINT_LOC_PUR << std::endl
#define PCLE std::cout << PRINT_LOC_CYN << std::endl

// PV is for print variable
#define PVEC(x, c) std::cout << c << #x << RST << "=" << x << std::endl
#define PVFC(x, c) std::cout << c << #x << RST << "=" << x << " "
#define PVCC(x, c) std::cout << c << #x << RST << "=" << x << ", "
#define PVSC(x, c) std::cout << c << #x << RST << "=" << x << "; "
#define PVDC(x, y, c) std::cout << c << #x << RST << "=" << x << y << " "

// PS is for print string, which can be a variable too, but it's name won't be
// printed, just the value.
#define PSEC(x, c) std::cout << c << x << RST << std::endl
#define PSFC(x, c) std::cout << c << x << RST << " "
#define PSCC(x, c) std::cout << c << x << RST << ", "
#define PSSC(x, c) std::cout << c << x << RST << "; "
#define PSDC(x, y, c) std::cout << c << x << RST << y << " "

//Print Matrix.
#define PMC(x, c) std::cout << c << #x << RST << std::endl << x << std::endl
#define PMLC(x, c)                                                             \
  std::cout << PRINT_LOC_##c << std::endl;                                     \
  PMC(x, c)
#define PMLCC(x, c1, c2)                                                       \
  std::cout << PRINT_LOC_##c1 << std::endl;                                    \
  PMC(x, c2)

#define PVE(x) PVEC(x, RST)
#define PVF(x) PVFC(x, RST)
#define PVC(x) PVCC(x, RST)
#define PVS(x) PVSC(x, RST)
#define PVD(x, y) PVDC(x, y, RST)

#define PSE(x) PSEC(x, RST)
#define PSF(x) PSFC(x, RST)
#define PSC(x) PSCC(x, RST)
#define PSS(x) PSSC(x, RST)
#define PSD(x, y) PSDC(x, y, RST)

#define PVELC(x, c)                                                            \
  std::cout << PRINT_LOC_DGR;                                                  \
  PVEC(x, c)
#define PVFLC(x, c)                                                            \
  std::cout << PRINT_LOC_DGR;                                                  \
  PVFC(x, c)
#define PVCLC(x, c)                                                            \
  std::cout << PRINT_LOC_DGR;                                                  \
  PVCC(x, c)
#define PVDLC(x, c)                                                            \
  std::cout << PRINT_LOC_DGR;                                                  \
  PVDC(x, c)
//#define PVELCC(x,c1,c2) std::cout << PRINT_LOC_##c1; PVEC(x,c2)
//#define PVFLCC(x,c1,c2) std::cout << PRINT_LOC_##c1; PVFC(x,c2)
//#define PVCLCC(x,c1,c2) std::cout << PRINT_LOC_##c1; PVCC(x,c2)
//#define PVDLCC(x,y,c1,c2) std::cout << PRINT_LOC_##c1; PVDC(x,y,c2)

#define PSELC(x, c)                                                            \
  std::cout << PRINT_LOC_DGR;                                                  \
  PSE(x, c)
#define PSFLC(x, c)                                                            \
  std::cout << PRINT_LOC_DGR;                                                  \
  PSF(x, c)
#define PSCLC(x, c)                                                            \
  std::cout << PRINT_LOC_DGR;                                                  \
  PSC(x, c)
#define PSDLC(x, c)                                                            \
  std::cout << PRINT_LOC_DGR;                                                  \
  PSD(x, c)
//#define PSELCC(x,c1,c2) std::cout << PRINT_LOC_##c1; PSEC(x,c2)
//#define PSFLCC(x,c1,c2) std::cout << PRINT_LOC_##c1; PSFC(x,c2)
//#define PSCLCC(x,c1,c2) std::cout << PRINT_LOC_##c1; PSCC(x,c2)
//#define PSDLCC(x,y,c1,c2) std::cout << PRINT_LOC_##c1; PSDC(x,y,c2)

//Print conditionally
#define cPVE(cond, x)                                                          \
  if (cond)                                                                    \
  PVE(x)
#define cPVF(cond, x)                                                          \
  if (cond)                                                                    \
  PVF(x)
#define cPVC(cond, x)                                                          \
  if (cond)                                                                    \
  PVC(x)
#define cPVS(cond, x)                                                          \
  if (cond)                                                                    \
  PVS(x)
#define cPVD(cond, x, y)                                                       \
  if (cond)                                                                    \
  PVD(x, y)

#define cPSE(cond, x)                                                          \
  if (cond)                                                                    \
  PSE(x)
#define cPSF(cond, x)                                                          \
  if (cond)                                                                    \
  PSF(x)
#define cPSC(cond, x)                                                          \
  if (cond)                                                                    \
  PSC(x)
#define cPSS(cond, x)                                                          \
  if (cond)                                                                    \
  PSS(x)
#define cPSD(cond, x, y)                                                       \
  if (cond)                                                                    \
  PSD(x, y)

#define cPVEC(cond, x, c)                                                      \
  if (cond)                                                                    \
  PVEC(x, c)
#define cPVFC(cond, x, c)                                                      \
  if (cond)                                                                    \
  PVFC(x, c)
#define cPVCC(cond, x, c)                                                      \
  if (cond)                                                                    \
  PVCC(x, c)
#define cPVSC(cond, x, c)                                                      \
  if (cond)                                                                    \
  PVSC(x, c)
#define cPVDC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
  PVDC(x, y, c)

#define cPSEC(cond, x, c)                                                      \
  if (cond)                                                                    \
  PSEC(x, c)
#define cPSFC(cond, x, c)                                                      \
  if (cond)                                                                    \
  PSFC(x, c)
#define cPSCC(cond, x, c)                                                      \
  if (cond)                                                                    \
  PSCC(x, c)
#define cPSSC(cond, x, c)                                                      \
  if (cond)                                                                    \
  PSSC(x, c)
#define cPSDC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
  PSDC(x, y, c)

#define cPVELC(cond, x, c)                                                     \
  cPSF(cond, PRINT_LOC_##c);                                                   \
  cPVE(cond, x)
#define cPVFLC(cond, x, c)                                                     \
  cPSF(cond, PRINT_LOC_##c);                                                   \
  cPVF(cond, x)
#define cPVCLC(cond, x, c)                                                     \
  cPSF(cond, PRINT_LOC_##c);                                                   \
  cPVC(cond, x)
#define cPVDLC(cond, x, c)                                                     \
  cPSF(cond, PRINT_LOC_##c);                                                   \
  cPVD(cond, x)
#define cPVELCC(cond, x, c1, c2)                                               \
  cPSF(cond, PRINT_LOC_##c1);                                                  \
  cPVEC(cond, x, c2)
#define cPVFLCC(cond, x, c1, c2)                                               \
  cPSF(cond, PRINT_LOC_##c1);                                                  \
  cPVFC(cond, x, c2)
#define cPVCLCC(cond, x, c1, c2)                                               \
  cPSF(cond, PRINT_LOC_##c1);                                                  \
  cPVCC(cond, x, c2)
#define cPVDLCC(cond, x, y, c1, c2)                                            \
  cPSF(cond, PRINT_LOC_##c1);                                                  \
  cPVDC(cond, x, y, c2)

#define cPSELC(cond, x, c)                                                     \
  cPSF(cond, PRINT_LOC_DGR);                                                   \
  cPSE(cond, x, c)
#define cPSFLC(cond, x, c)                                                     \
  cPSF(cond, PRINT_LOC_DGR);                                                   \
  cPSF(cond, x, c)
#define cPSCLC(cond, x, c)                                                     \
  cPSF(cond, PRINT_LOC_DGR);                                                   \
  cPSC(cond, x, c)
#define cPSDLC(cond, x, c)                                                     \
  cPSF(cond, PRINT_LOC_DGR);                                                   \
  cPSD(cond, x, c)
#define cPSELCC(cond, x, c1, c2)                                               \
  cPSF(cond, PRINT_LOC_##c1);                                                  \
  cPSEC(cond, x, c2)
#define cPSFLCC(cond, x, c1, c2)                                               \
  cPSF(cond, PRINT_LOC_##c1);                                                  \
  cPSFC(cond, x, c2)
#define cPSCLCC(cond, x, c1, c2)                                               \
  cPSF(cond, PRINT_LOC_##c1);                                                  \
  cPSCC(cond, x, c2)
#define cPSDLCC(cond, x, y, c1, c2)                                            \
  cPSF(cond, PRINT_LOC_##c1);                                                  \
  cPSDC(cond, x, y, c2)

#define CPVE(cond, x, y)                                                       \
  if (cond)                                                                    \
    PVE(x);                                                                    \
  else                                                                         \
  PVE(y)
#define CPVF(cond, x, y)                                                       \
  if (cond)                                                                    \
    PVF(x);                                                                    \
  else                                                                         \
  PVE(y)
#define CPVC(cond, x, y)                                                       \
  if (cond)                                                                    \
    PVC(x);                                                                    \
  else                                                                         \
  PVE(y)
#define CPVS(cond, x, y)                                                       \
  if (cond)                                                                    \
    PVS(x);                                                                    \
  else                                                                         \
  PVE(y)
#define CPVD(cond, x, y, z)                                                    \
  if (cond)                                                                    \
    PVD(x, z);                                                                 \
  else                                                                         \
  PVD(y, z)

#define CPSE(cond, x, y)                                                       \
  if (cond)                                                                    \
    PSE(x);                                                                    \
  else                                                                         \
  PSE(y)
#define CPSF(cond, x, y)                                                       \
  if (cond)                                                                    \
    PSF(x);                                                                    \
  else                                                                         \
  PSF(y)
#define CPSC(cond, x, y)                                                       \
  if (cond)                                                                    \
    PSC(x);                                                                    \
  else                                                                         \
  PSC(y)
#define CPSS(cond, x, y)                                                       \
  if (cond)                                                                    \
    PSS(x);                                                                    \
  else                                                                         \
  PSS(y)
#define CPSD(cond, x, y, z)                                                    \
  if (cond)                                                                    \
    PSD(x, z);                                                                 \
  else                                                                         \
  PSD(y, z)

#define CPVEC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
    PVEC(x, c);                                                                \
  else                                                                         \
  PVEC(y, c)
#define CPVFC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
    PVFC(x, c);                                                                \
  else                                                                         \
  PVFC(y, c)
#define CPVCC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
    PVCC(x, c);                                                                \
  else                                                                         \
  PVCC(y, c)
#define CPVSC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
    PVSC(x, c);                                                                \
  else                                                                         \
  PVSC(y, c)
#define CPVDC(cond, x, y, z, c)                                                \
  if (cond)                                                                    \
    PVDC(x, z, c);                                                             \
  else                                                                         \
  PVDC(y, z, c)

#define CPSEC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
    PSEC(x, c);                                                                \
  else                                                                         \
  PSEC(y, c)
#define CPSFC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
    PSFC(x, c);                                                                \
  else                                                                         \
  PSFC(y, c)
#define CPSCC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
    PSCC(x, c);                                                                \
  else                                                                         \
  PSCC(y, c)
#define CPSSC(cond, x, y, c)                                                   \
  if (cond)                                                                    \
    PSSC(x, c);                                                                \
  else                                                                         \
  PSSC(y, c)
#define CPSDC(cond, x, y, z, c)                                                \
  if (cond)                                                                    \
    PSDC(x, z, c);                                                             \
  else                                                                         \
  PSDC(y, z, c)

#define CPVECC(cond, x, y, c1, c2)                                             \
  if (cond)                                                                    \
    PVEC(x, c1);                                                               \
  else                                                                         \
  PVEC(y, c2)
#define CPVFCC(cond, x, y, c1, c2)                                             \
  if (cond)                                                                    \
    PVFC(x, c1);                                                               \
  else                                                                         \
  PVFC(y, c2)
#define CPVCCC(cond, x, y, c1, c2)                                             \
  if (cond)                                                                    \
    PVCC(x, c1);                                                               \
  else                                                                         \
  PVCC(y, c2)
#define CPVSCC(cond, x, y, c1, c2)                                             \
  if (cond)                                                                    \
    PVSC(x, c1);                                                               \
  else                                                                         \
  PVSC(y, c2)
#define CPVDCC(cond, x, y, z, c1, c2)                                          \
  if (cond)                                                                    \
    PVDC(x, z, c1);                                                            \
  else                                                                         \
  PVDC(y, z, c2)

#define CPSECC(cond, x, y, c1, c2)                                             \
  if (cond)                                                                    \
    PSEC(x, c1);                                                               \
  else                                                                         \
  PSEC(y, c2)
#define CPSFCC(cond, x, y, c1, c2)                                             \
  if (cond)                                                                    \
    PSFC(x, c1);                                                               \
  else                                                                         \
  PSFC(y, c2)
#define CPSCCC(cond, x, y, c1, c2)                                             \
  if (cond)                                                                    \
    PSCC(x, c1);                                                               \
  else                                                                         \
  PSCC(y, c2)
#define CPSSCC(cond, x, y, c1, c2)                                             \
  if (cond)                                                                    \
    PSSC(x, c1);                                                               \
  else                                                                         \
  PSSC(y, c2)
#define CPSDCC(cond, x, y, z, c1, c2)                                          \
  if (cond)                                                                    \
    PSDC(x, z, c1);                                                            \
  else                                                                         \
  PSDC(y, z, c2)

#define CPVELC(cond, x, y, c)                                                  \
  PDL;                                                                         \
  CPVE(cond, x)
#define CPVFLC(cond, x, y, c)                                                  \
  PDL;                                                                         \
  CPVF(cond, x)
#define CPVCLC(cond, x, y, c)                                                  \
  PDL;                                                                         \
  CPVC(cond, x)
#define CPVDLC(cond, x, y, c)                                                  \
  PDL;                                                                         \
  CPVD(cond, x)
#define CPVELCC(cond, x, y, c1, c2)                                            \
  std::cout << PRINT_LOC_##c1;                                                 \
  CPVEC(cond, x, c2)
#define CPVFLCC(cond, x, y, c1, c2)                                            \
  std::cout << PRINT_LOC_##c1;                                                 \
  CPVFC(cond, x, c2)
#define CPVCLCC(cond, x, y, c1, c2)                                            \
  std::cout << PRINT_LOC_##c1;                                                 \
  CPVCC(cond, x, c2)
#define CPVDLCC(cond, x, y, z, c1, c2)                                         \
  std::cout << PRINT_LOC_##c1;                                                 \
  CPVDC(cond, x, y, z, c2)

#define CPSELC(cond, x, y, c)                                                  \
  PDL;                                                                         \
  CPSE(cond, x, y)
#define CPSFLC(cond, x, y, c)                                                  \
  PDL;                                                                         \
  CPSF(cond, x, y)
#define CPSCLC(cond, x, y, c)                                                  \
  PDL;                                                                         \
  CPSC(cond, x, y)
#define CPSDLC(cond, x, y, c)                                                  \
  PDL;                                                                         \
  CPSD(cond, x, y)
#define CPSELCC(cond, x, y, c1, c2)                                            \
  std::cout << PRINT_LOC_##c1;                                                 \
  CPSEC(cond, x, y, c2)
#define CPSFLCC(cond, x, y, c1, c2)                                            \
  std::cout << PRINT_LOC_##c1;                                                 \
  CPSFC(cond, x, y, c2)
#define CPSCLCC(cond, x, y, c1, c2)                                            \
  std::cout << PRINT_LOC_##c1;                                                 \
  CPSCC(cond, x, y, c2)
#define CPSDLCC(cond, x, y, z, c1, c2)                                         \
  std::cout << PRINT_LOC_##c1;                                                 \
  CPSDC(cond, x, y, z, c2)

//Print only in DEBUG mode

#ifndef NDEBUG
#define DBG 1
#else
#define DBG 0
#endif

#define DPVE(x)                                                                \
  if (DBG)                                                                     \
  PVE(x)
#define DPVF(x)                                                                \
  if (DBG)                                                                     \
  PVF(x)
#define DPVC(x)                                                                \
  if (DBG)                                                                     \
  PVC(x)
#define DPVS(x)                                                                \
  if (DBG)                                                                     \
  PVS(x)
#define DPVD(x, y)                                                             \
  if (DBG)                                                                     \
  PVD(x, y)

#define DPSE(x)                                                                \
  if (DBG)                                                                     \
  PSE(x)
#define DPSF(x)                                                                \
  if (DBG)                                                                     \
  PSF(x)
#define DPSC(x)                                                                \
  if (DBG)                                                                     \
  PSC(x)
#define DPSS(x)                                                                \
  if (DBG)                                                                     \
  PSS(x)
#define DPSD(x, y)                                                             \
  if (DBG)                                                                     \
  PSD(x, y)

#define DPVEC(x, c)                                                            \
  if (DBG)                                                                     \
  PVEC(x, c)
#define DPVFC(x, c)                                                            \
  if (DBG)                                                                     \
  PVFC(x, c)
#define DPVCC(x, c)                                                            \
  if (DBG)                                                                     \
  PVCC(x, c)
#define DPVSC(x, c)                                                            \
  if (DBG)                                                                     \
  PVSC(x, c)
#define DPVDC(x, y, c)                                                         \
  if (DBG)                                                                     \
  PVDC(x, y, c)

#define DPSEC(x, c)                                                            \
  if (DBG)                                                                     \
  PSEC(x, c)
#define DPSFC(x, c)                                                            \
  if (DBG)                                                                     \
  PSFC(x, c)
#define DPSCC(x, c)                                                            \
  if (DBG)                                                                     \
  PSCC(x, c)
#define DPSSC(x, c)                                                            \
  if (DBG)                                                                     \
  PSSC(x, c)
#define DPSDC(x, y, c)                                                         \
  if (DBG)                                                                     \
  PSDC(x, y, c)

#define DPVELC(x, c)                                                           \
  cPSF(DBG, PRINT_LOC_##c);                                                    \
  cPVE(DBG, x)
#define DPVFLC(x, c)                                                           \
  cPSF(DBG, PRINT_LOC_##c);                                                    \
  cPVF(DBG, x)
#define DPVCLC(x, c)                                                           \
  cPSF(DBG, PRINT_LOC_##c);                                                    \
  cPVC(DBG, x)
#define DPVDLC(x, c)                                                           \
  cPSF(DBG, PRINT_LOC_##c);                                                    \
  cPVD(DBG, x)
#define DPVELCC(x, c1, c2)                                                     \
  cPSF(DBG, PRINT_LOC_##c1);                                                   \
  cPVEC(DBG, x, c2)
#define DPVFLCC(x, c1, c2)                                                     \
  cPSF(DBG, PRINT_LOC_##c1);                                                   \
  cPVFC(DBG, x, c2)
#define DPVCLCC(x, c1, c2)                                                     \
  cPSF(DBG, PRINT_LOC_##c1);                                                   \
  cPVCC(DBG, x, c2)
#define DPVDLCC(x, y, c1, c2)                                                  \
  cPSF(DBG, PRINT_LOC_##c1);                                                   \
  cPVDC(DBG, x, y, c2)

#define DPSELC(x, c)                                                           \
  cPSF(DBG, PRINT_LOC_DGR);                                                    \
  cPSE(DBG, x, c)
#define DPSFLC(x, c)                                                           \
  cPSF(DBG, PRINT_LOC_DGR);                                                    \
  cPSF(DBG, x, c)
#define DPSCLC(x, c)                                                           \
  cPSF(DBG, PRINT_LOC_DGR);                                                    \
  cPSC(DBG, x, c)
#define DPSDLC(x, c)                                                           \
  cPSF(DBG, PRINT_LOC_DGR);                                                    \
  cPSD(DBG, x, c)
#define DPSELCC(x, c1, c2)                                                     \
  cPSF(DBG, PRINT_LOC_##c1);                                                   \
  cPSEC(DBG, x, c2)
#define DPSFLCC(x, c1, c2)                                                     \
  cPSF(DBG, PRINT_LOC_##c1);                                                   \
  cPSFC(DBG, x, c2)
#define DPSCLCC(x, c1, c2)                                                     \
  cPSF(DBG, PRINT_LOC_##c1);                                                   \
  cPSCC(DBG, x, c2)
#define DPSDLCC(x, y, c1, c2)                                                  \
  cPSF(DBG, PRINT_LOC_##c1);                                                   \
  cPSDC(DBG, x, y, c2)


//Print Vector.
#define PRINT_VECTOR(x, d, c1, c2)                                             \
  std::cout << PRINT_LOC_##c1 << "\n" << c2 << #x << "(" << x.size() << "):";  \
  CPSEC(!x.size(), " Empty Vec", "", RED);                                     \
  for (size_t ii = 1; ii < x.size(); ii++)                                     \
    PSD(x[ii - 1], d);                                                         \
  CPSE(x.size(), x[x.size() - 1], "");

// Shared pointer declares
#ifndef S_V(x)
#define S_V(x) *x.get()
#endif

#endif /* end of include guard: DEFS_H_BKFT10EO */
#endif
