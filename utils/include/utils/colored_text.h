#pragma once

#include <string>

namespace ct {
// Colors for printing
const std::string RST = "\E[0m";

// FG colors
const std::string GRY =
    "\E[1;30m"; // Dull grey#define GRY "\E[1;30m" //Dull grey
const std::string BLK = "\E[0;30m"; // Black#define BLK "\E[0;30m" //Black
const std::string DRD = "\E[0;31m"; // Dull Red#define DRD "\E[0;31m" //Dull Red
const std::string RED = "\E[1;31m"; // Red#define RED "\E[1;31m" //Red
const std::string DGN =
    "\E[0;32m"; // Dull Green #define DGN "\E[0;32m" //Dull Green
const std::string GRN = "\E[1;32m"; // Green #define GRN "\E[1;32m" //Green
const std::string DBU =
    "\E[0;34m"; // Dull Blue#define DBU "\E[0;34m" //Dull Blue
const std::string BLU = "\E[1;34m"; // Blue#define BLU "\E[1;34m" //Blue
const std::string DYW =
    "\E[0;33m"; // Dull Yellow#define DYW "\E[0;33m" //Dull Yellow
const std::string YLW = "\E[1;33m"; // Yellow#define YLW "\E[1;33m" //Yellow
const std::string DPR = "\E[0;35m"; // Dull Magenta or puprle#define DPR
                                    // "\E[0;35m" //Dull Magenta or puprle
const std::string PUR =
    "\E[1;35m"; // Magenta or puprle#define PUR "\E[1;35m" //Magenta or puprle
const std::string DCN =
    "\E[0;36m"; // Dull Cyan#define DCN "\E[0;36m" //Dull Cyan
const std::string CYN = "\E[1;36m"; // Cyan#define CYN "\E[1;36m" //Cyan
const std::string DWT = "\E[0;37m"; // White#define DWT "\E[0;37m" //White
const std::string WHT = "\E[1;37m"; // White#define WHT "\E[1;37m" //White

// BG colors
const std::string BBLK = "\E[0;40m"; // Black
const std::string BGRY = "\E[1;40m"; // Dull grey
const std::string BDRD = "\E[0;41m"; // Dull Red
const std::string BRED = "\E[1;41m"; // Red
const std::string BDGN = "\E[0;42m"; // Dull Green
const std::string BGRN = "\E[1;42m"; // Green
const std::string BDBU = "\E[0;44m"; // Dull Blue
const std::string BBLU = "\E[1;44m"; // Blue
const std::string BDYW = "\E[0;44m"; // Dull Yellow
const std::string BYLW = "\E[1;44m"; // Yellow
const std::string BDPR = "\E[0;45m"; // Dull Magenta or puprle
const std::string BPUR = "\E[1;45m"; // Magenta or puprle
const std::string BDCN = "\E[0;46m"; // Dull Cyan
const std::string BCYN = "\E[1;46m"; // Cyan
const std::string BDWT = "\E[0;47m"; // White
const std::string BWHT = "\E[1;47m"; // White

} /* ct */ /*  ct */

// d is for dull
// NOTE: for error at ct::RED or red, since red is a generic word (unlike grn,
// blu or ylw) it might result in conflicts and this macro might result in
// incorrect text replacement in resulting translation unit with syntax error of
// some kind. In such a scenario ensure this file is included after the external
// library header has been included.
// e.g.:
//<file:<line>: >error: invalid use of ‘::’
//#define red(x) ct::RED << x << ct::RST
#define ct_red(x) ct::RED << x << ct::RST
#define ct_blk(x) ct::BLK << x << ct::RST
#define ct_grn(x) ct::GRN << x << ct::RST
#define ct_blu(x) ct::BLU << x << ct::RST
#define ct_ylw(x) ct::YLW << x << ct::RST
#define ct_cyn(x) ct::CYN << x << ct::RST
#define ct_pur(x) ct::PUR << x << ct::RST
#define ct_wht(x) ct::WHT << x << ct::RST
#define ct_rst(x) ct::RST << x << ct::RST
#define ct_dgry(x) ct::GRY << x << ct::RST
#define ct_dred(x) ct::DRD << x << ct::RST
#define ct_dgrn(x) ct::DGN << x << ct::RST
#define ct_dblu(x) ct::DBU << x << ct::RST
#define ct_dylw(x) ct::DYW << x << ct::RST
#define ct_dcyn(x) ct::DCN << x << ct::RST
#define ct_dpur(x) ct::DPR << x << ct::RST
#define ct_dwht(x) ct::DWT << x << ct::RST

// v is for variable, d is for dull
#define ct_vblk(x) ct_blk(#x) << "=" << x
#define ct_vred(x) ct_red(#x) << "=" << x
#define ct_vgrn(x) ct_grn(#x) << "=" << x
#define ct_vblu(x) ct_blu(#x) << "=" << x
#define ct_vylw(x) ct_ylw(#x) << "=" << x
#define ct_vcyn(x) ct_cyn(#x) << "=" << x
#define ct_vpur(x) ct_pur(#x) << "=" << x
#define ct_vwht(x) ct_wht(#x) << "=" << x
#define ct_vdgry(x) ct_dgry(#x) << "=" << x
#define ct_vdred(x) ct_dred(#x) << "=" << x
#define ct_vdgrn(x) ct_dgrn(#x) << "=" << x
#define ct_vdblu(x) ct_dblu(#x) << "=" << x
#define ct_vdylw(x) ct_dylw(#x) << "=" << x
#define ct_vdcyn(x) ct_dcyn(#x) << "=" << x
#define ct_vdpur(x) ct_dpur(#x) << "=" << x
#define ct_vdwht(x) ct_dwht(#x) << "=" << x

// b is for background, d is for dull
#define ct_bblk(x) ct::BBLK << x << ct::RST
#define ct_bred(x) ct::BRED << x << ct::RST
#define ct_bgrn(x) ct::BGRN << x << ct::RST
#define ct_bblu(x) ct::BBLU << x << ct::RST
#define ct_bylw(x) ct::BYLW << x << ct::RST
#define ct_bcyn(x) ct::BCYN << x << ct::RST
#define ct_bpur(x) ct::BPUR << x << ct::RST
#define ct_bwht(x) ct::BWHT << x << ct::RST
#define ct_bdgry(x) ct::BGRY << x << ct::RST
#define ct_bdred(x) ct::BDRD << x << ct::RST
#define ct_bdgrn(x) ct::BDGN << x << ct::RST
#define ct_bdblu(x) ct::BDBU << x << ct::RST
#define ct_bdylw(x) ct::BDYW << x << ct::RST
#define ct_bdcyn(x) ct::BDCN << x << ct::RST
#define ct_bdpur(x) ct::BDPR << x << ct::RST
#define ct_bdwht(x) ct::BDWT << x << ct::RST

// v is for variable, b is for background, d is for dull
#define ct_vbblk(x) ct_bblk(#x) << "=" << x
#define ct_vbred(x) ct_bred(#x) << "=" << x
#define ct_vbgrn(x) ct_bgrn(#x) << "=" << x
#define ct_vbblu(x) ct_bblu(#x) << "=" << x
#define ct_vbylw(x) ct_bylw(#x) << "=" << x
#define ct_vbcyn(x) ct_bcyn(#x) << "=" << x
#define ct_vbpur(x) ct_bpur(#x) << "=" << x
#define ct_vbwht(x) ct_bwht(#x) << "=" << x
#define ct_vbdgry(x) ct_bdgry(#x) << "=" << x
#define ct_vbdred(x) ct_bdred(#x) << "=" << x
#define ct_vbdgrn(x) ct_bdgrn(#x) << "=" << x
#define ct_vbdblu(x) ct_bdblu(#x) << "=" << x
#define ct_vbdylw(x) ct_bdylw(#x) << "=" << x
#define ct_vbdcyn(x) ct_bdcyn(#x) << "=" << x
#define ct_vbdpur(x) ct_bdpur(#x) << "=" << x
#define ct_vbdwht(x) ct_bdwht(#x) << "=" << x
