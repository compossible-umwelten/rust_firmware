#![allow(dead_code)]
/* Main Register Map */
/* section 6.4 Register Map. Table 10 */
pub const REFERENCE /* ----- */: u8 = 0x0; // 6.4.1 Table 12
pub const ION_BL /* -------- */: u8 = 0x1; // 6.4.1 Table 13
pub const DEADTIME /* ------ */: u8 = 0x2; // 6.4.1 Table 14
pub const KP /* ------------ */: u8 = 0x3; // 6.4.1 Table 15
pub const KPA_KI /* -------- */: u8 = 0x4; // 6.4.1 Table 16
pub const CONFIG /* -------- */: u8 = 0x5; // 6.4.1 Table 17
pub const PARCAP /* -------- */: u8 = 0x6; // 6.4.1 Table 18
pub const SUP_RISE /* ------ */: u8 = 0x7; // 6.4.1 Table 19
pub const DAC /* ----------- */: u8 = 0x8; // 6.4.1 Table 20
pub const IC_STATUS /* ----- */: u8 = 0xC; // 6.4.1 Table 21
pub const SENSE /* --------- */: u8 = 0xD; // 6.4.1 Table 22
pub const TRIM /* ---------- */: u8 = 0xE; // 6.4.1 Table 23


/* Broadcast Register Map */
/* section 6.4 Register Map. Table 11 */
pub const BROADCAST_0X0 /* -------- */: u8 = 0x0;
pub const BROADCAST_0X1 /* -------- */: u8 = 0x1;
pub const BROADCAST_0X2 /* -------- */: u8 = 0x2;
pub const BROADCAST_0X3 /* -------- */: u8 = 0x3;
pub const BROADCAST_0X4 /* -------- */: u8 = 0x4;
pub const BROADCAST_0X5 /* -------- */: u8 = 0x5;
pub const BROADCAST_0X6 /* -------- */: u8 = 0x6;
pub const BROADCAST_0X7 /* -------- */: u8 = 0x7;
pub const BROADCAST_0X8 /* -------- */: u8 = 0x8;
pub const BROADCAST_0X9 /* -------- */: u8 = 0x9;
pub const BROADCAST_0XA /* -------- */: u8 = 0xA;
pub const BROADCAST_0XB /* -------- */: u8 = 0xB;
pub const BROADCAST_0XC /* -------- */: u8 = 0xC;
pub const BROADCAST_0XD /* -------- */: u8 = 0xD;
pub const BROADCAST_0XE /* -------- */: u8 = 0xE;

pub const BROADCAST_0X10 /*-------- */: u8 = 0x10;
pub const BROADCAST_0X12 /*-------- */: u8 = 0x12;
pub const BROADCAST_0X13 /*-------- */: u8 = 0x13;
pub const BROADCAST_0X14 /*-------- */: u8 = 0x14;
pub const BROADCAST_0X1A /*-------- */: u8 = 0x1A;
