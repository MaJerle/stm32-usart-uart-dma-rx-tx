/**
  **********************************************************************************************************************
  * @file   stm32c5xx_ll_ramcfg.h
  * @brief  Header file of RAMCFG LL module.
  **********************************************************************************************************************
    * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********************************************************************************************************************
  */
/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_LL_RAMCFG_H
#define STM32C5XX_LL_RAMCFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

/** @addtogroup RAMCFG_LL RAMCFG
  * @{
  */
/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup RAMCFG_LL_Exported_Constants  LL RAMCFG Constants
  * @{
  */

/** @defgroup RAMCFG_FLAG RAMCFG Monitor Flags
  * @{
  */
#define LL_RAMCFG_FLAG_SRAMBUSY  RAMCFG_ISR_SRAMBUSY    /*!< RAMCFG SRAM busy flag                               */
#define LL_RAMCFG_FLAG_SEDC      RAMCFG_ISR_SEDC        /*!< RAMCFG ECC single error detected and corrected flag */
#define LL_RAMCFG_FLAG_DED       RAMCFG_ISR_DED         /*!< RAMCFG ECC double error detected flag               */
#define LL_RAMCFG_FLAG_ECC_ALL   (LL_RAMCFG_FLAG_SEDC \
                                  | LL_RAMCFG_FLAG_DED)  /*!< RAMCFG all ECC error flags                         */
/**
  * @}
  */

/** @defgroup RAMCFG_Interrupt RAMCFG Interrupts
  * @{
  */
#define LL_RAMCFG_IT_SE  RAMCFG_IER_SEIE                /*!< RAMCFG ECC single error interrupt                   */
#define LL_RAMCFG_IT_DE  RAMCFG_IER_DEIE                /*!< RAMCFG ECC double error interrupt                   */
#define LL_RAMCFG_IT_NMI RAMCFG_IER_ECCNMI              /*!< RAMCFG ECC double error redirected to NMI interrupt */
#define LL_RAMCFG_IT_ALL (LL_RAMCFG_IT_SE \
                          | LL_RAMCFG_IT_DE \
                          | LL_RAMCFG_IT_NMI)           /*!< RAMCFG all RAMCFG interrupts                        */
/**
  * @}
  */


/** @defgroup RAMCFG_WPR_Page_Attr_Selection RAMCFG Write protection page Attr Selection
  * @{
  */
#define LL_RAMCFG_WRP_PAGE_0   RAMCFG_WPR1_P0WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 0  */
#define LL_RAMCFG_WRP_PAGE_1   RAMCFG_WPR1_P1WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 1  */
#define LL_RAMCFG_WRP_PAGE_2   RAMCFG_WPR1_P2WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 2  */
#define LL_RAMCFG_WRP_PAGE_3   RAMCFG_WPR1_P3WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 3  */
#define LL_RAMCFG_WRP_PAGE_4   RAMCFG_WPR1_P4WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 4  */
#define LL_RAMCFG_WRP_PAGE_5   RAMCFG_WPR1_P5WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 5  */
#define LL_RAMCFG_WRP_PAGE_6   RAMCFG_WPR1_P6WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 6  */
#define LL_RAMCFG_WRP_PAGE_7   RAMCFG_WPR1_P7WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 7  */
#define LL_RAMCFG_WRP_PAGE_8   RAMCFG_WPR1_P8WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 8  */
#define LL_RAMCFG_WRP_PAGE_9   RAMCFG_WPR1_P9WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 9  */
#define LL_RAMCFG_WRP_PAGE_10  RAMCFG_WPR1_P10WP /*!< LL RAMCFG WRITE PROTECTION PAGE 10 */
#define LL_RAMCFG_WRP_PAGE_11  RAMCFG_WPR1_P11WP /*!< LL RAMCFG WRITE PROTECTION PAGE 11 */
#define LL_RAMCFG_WRP_PAGE_12  RAMCFG_WPR1_P12WP /*!< LL RAMCFG WRITE PROTECTION PAGE 12 */
#define LL_RAMCFG_WRP_PAGE_13  RAMCFG_WPR1_P13WP /*!< LL RAMCFG WRITE PROTECTION PAGE 13 */
#define LL_RAMCFG_WRP_PAGE_14  RAMCFG_WPR1_P14WP /*!< LL RAMCFG WRITE PROTECTION PAGE 14 */
#define LL_RAMCFG_WRP_PAGE_15  RAMCFG_WPR1_P15WP /*!< LL RAMCFG WRITE PROTECTION PAGE 15 */
#define LL_RAMCFG_WRP_PAGE_16  RAMCFG_WPR1_P16WP /*!< LL RAMCFG WRITE PROTECTION PAGE 16 */
#define LL_RAMCFG_WRP_PAGE_17  RAMCFG_WPR1_P17WP /*!< LL RAMCFG WRITE PROTECTION PAGE 17 */
#define LL_RAMCFG_WRP_PAGE_18  RAMCFG_WPR1_P18WP /*!< LL RAMCFG WRITE PROTECTION PAGE 18 */
#define LL_RAMCFG_WRP_PAGE_19  RAMCFG_WPR1_P19WP /*!< LL RAMCFG WRITE PROTECTION PAGE 19 */
#if defined(RAMCFG_WPR1_P20WP)
#define LL_RAMCFG_WRP_PAGE_20  RAMCFG_WPR1_P20WP /*!< LL RAMCFG WRITE PROTECTION PAGE 20 */
#define LL_RAMCFG_WRP_PAGE_21  RAMCFG_WPR1_P21WP /*!< LL RAMCFG WRITE PROTECTION PAGE 21 */
#define LL_RAMCFG_WRP_PAGE_22  RAMCFG_WPR1_P22WP /*!< LL RAMCFG WRITE PROTECTION PAGE 22 */
#define LL_RAMCFG_WRP_PAGE_23  RAMCFG_WPR1_P23WP /*!< LL RAMCFG WRITE PROTECTION PAGE 23 */
#define LL_RAMCFG_WRP_PAGE_24  RAMCFG_WPR1_P24WP /*!< LL RAMCFG WRITE PROTECTION PAGE 24 */
#define LL_RAMCFG_WRP_PAGE_25  RAMCFG_WPR1_P25WP /*!< LL RAMCFG WRITE PROTECTION PAGE 25 */
#define LL_RAMCFG_WRP_PAGE_26  RAMCFG_WPR1_P26WP /*!< LL RAMCFG WRITE PROTECTION PAGE 26 */
#define LL_RAMCFG_WRP_PAGE_27  RAMCFG_WPR1_P27WP /*!< LL RAMCFG WRITE PROTECTION PAGE 27 */
#define LL_RAMCFG_WRP_PAGE_28  RAMCFG_WPR1_P28WP /*!< LL RAMCFG WRITE PROTECTION PAGE 28 */
#define LL_RAMCFG_WRP_PAGE_29  RAMCFG_WPR1_P29WP /*!< LL RAMCFG WRITE PROTECTION PAGE 29 */
#define LL_RAMCFG_WRP_PAGE_30  RAMCFG_WPR1_P30WP /*!< LL RAMCFG WRITE PROTECTION PAGE 30 */
#define LL_RAMCFG_WRP_PAGE_31  RAMCFG_WPR1_P31WP /*!< LL RAMCFG WRITE PROTECTION PAGE 31 */
#endif /* RAMCFG_WPR1_P20WP */
#if defined(RAMCFG_WPR2_P32WP)
#define LL_RAMCFG_WRP_PAGE_32  RAMCFG_WPR2_P32WP /*!< LL RAMCFG WRITE PROTECTION PAGE 32 */
#define LL_RAMCFG_WRP_PAGE_33  RAMCFG_WPR2_P33WP /*!< LL RAMCFG WRITE PROTECTION PAGE 33 */
#define LL_RAMCFG_WRP_PAGE_34  RAMCFG_WPR2_P34WP /*!< LL RAMCFG WRITE PROTECTION PAGE 34 */
#define LL_RAMCFG_WRP_PAGE_35  RAMCFG_WPR2_P35WP /*!< LL RAMCFG WRITE PROTECTION PAGE 35 */
#define LL_RAMCFG_WRP_PAGE_36  RAMCFG_WPR2_P36WP /*!< LL RAMCFG WRITE PROTECTION PAGE 36 */
#define LL_RAMCFG_WRP_PAGE_37  RAMCFG_WPR2_P37WP /*!< LL RAMCFG WRITE PROTECTION PAGE 37 */
#define LL_RAMCFG_WRP_PAGE_38  RAMCFG_WPR2_P38WP /*!< LL RAMCFG WRITE PROTECTION PAGE 38 */
#define LL_RAMCFG_WRP_PAGE_39  RAMCFG_WPR2_P39WP /*!< LL RAMCFG WRITE PROTECTION PAGE 39 */
#define LL_RAMCFG_WRP_PAGE_40  RAMCFG_WPR2_P40WP /*!< LL RAMCFG WRITE PROTECTION PAGE 40 */
#define LL_RAMCFG_WRP_PAGE_41  RAMCFG_WPR2_P41WP /*!< LL RAMCFG WRITE PROTECTION PAGE 41 */
#define LL_RAMCFG_WRP_PAGE_42  RAMCFG_WPR2_P42WP /*!< LL RAMCFG WRITE PROTECTION PAGE 42 */
#define LL_RAMCFG_WRP_PAGE_43  RAMCFG_WPR2_P43WP /*!< LL RAMCFG WRITE PROTECTION PAGE 43 */
#define LL_RAMCFG_WRP_PAGE_44  RAMCFG_WPR2_P44WP /*!< LL RAMCFG WRITE PROTECTION PAGE 44 */
#define LL_RAMCFG_WRP_PAGE_45  RAMCFG_WPR2_P45WP /*!< LL RAMCFG WRITE PROTECTION PAGE 45 */
#define LL_RAMCFG_WRP_PAGE_46  RAMCFG_WPR2_P46WP /*!< LL RAMCFG WRITE PROTECTION PAGE 46 */
#define LL_RAMCFG_WRP_PAGE_47  RAMCFG_WPR2_P47WP /*!< LL RAMCFG WRITE PROTECTION PAGE 47 */
#endif /* RAMCFG_WPR2_P32WP */
#if defined(RAMCFG_WPR2_P48WP)
#define LL_RAMCFG_WRP_PAGE_48  RAMCFG_WPR2_P48WP /*!< LL RAMCFG WRITE PROTECTION PAGE 48 */
#define LL_RAMCFG_WRP_PAGE_49  RAMCFG_WPR2_P49WP /*!< LL RAMCFG WRITE PROTECTION PAGE 49 */
#define LL_RAMCFG_WRP_PAGE_50  RAMCFG_WPR2_P50WP /*!< LL RAMCFG WRITE PROTECTION PAGE 50 */
#define LL_RAMCFG_WRP_PAGE_51  RAMCFG_WPR2_P51WP /*!< LL RAMCFG WRITE PROTECTION PAGE 51 */
#define LL_RAMCFG_WRP_PAGE_52  RAMCFG_WPR2_P52WP /*!< LL RAMCFG WRITE PROTECTION PAGE 52 */
#define LL_RAMCFG_WRP_PAGE_53  RAMCFG_WPR2_P53WP /*!< LL RAMCFG WRITE PROTECTION PAGE 53 */
#define LL_RAMCFG_WRP_PAGE_54  RAMCFG_WPR2_P54WP /*!< LL RAMCFG WRITE PROTECTION PAGE 54 */
#define LL_RAMCFG_WRP_PAGE_55  RAMCFG_WPR2_P55WP /*!< LL RAMCFG WRITE PROTECTION PAGE 55 */
#define LL_RAMCFG_WRP_PAGE_56  RAMCFG_WPR2_P56WP /*!< LL RAMCFG WRITE PROTECTION PAGE 56 */
#define LL_RAMCFG_WRP_PAGE_57  RAMCFG_WPR2_P57WP /*!< LL RAMCFG WRITE PROTECTION PAGE 57 */
#define LL_RAMCFG_WRP_PAGE_58  RAMCFG_WPR2_P58WP /*!< LL RAMCFG WRITE PROTECTION PAGE 58 */
#define LL_RAMCFG_WRP_PAGE_59  RAMCFG_WPR2_P59WP /*!< LL RAMCFG WRITE PROTECTION PAGE 59 */
#define LL_RAMCFG_WRP_PAGE_60  RAMCFG_WPR2_P60WP /*!< LL RAMCFG WRITE PROTECTION PAGE 60 */
#define LL_RAMCFG_WRP_PAGE_61  RAMCFG_WPR2_P61WP /*!< LL RAMCFG WRITE PROTECTION PAGE 61 */
#define LL_RAMCFG_WRP_PAGE_62  RAMCFG_WPR2_P62WP /*!< LL RAMCFG WRITE PROTECTION PAGE 62 */
#define LL_RAMCFG_WRP_PAGE_63  RAMCFG_WPR2_P63WP /*!< LL RAMCFG WRITE PROTECTION PAGE 63 */
#endif /* RAMCFG_WPR2_P48WP */
#if defined(RAMCFG_WPR3_P64WP)
#define LL_RAMCFG_WRP_PAGE_64  RAMCFG_WPR3_P64WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 64 */
#define LL_RAMCFG_WRP_PAGE_65  RAMCFG_WPR3_P65WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 65 */
#define LL_RAMCFG_WRP_PAGE_66  RAMCFG_WPR3_P66WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 66 */
#define LL_RAMCFG_WRP_PAGE_67  RAMCFG_WPR3_P67WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 67 */
#define LL_RAMCFG_WRP_PAGE_68  RAMCFG_WPR3_P68WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 68 */
#define LL_RAMCFG_WRP_PAGE_69  RAMCFG_WPR3_P69WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 69 */
#define LL_RAMCFG_WRP_PAGE_70  RAMCFG_WPR3_P70WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 70 */
#define LL_RAMCFG_WRP_PAGE_71  RAMCFG_WPR3_P71WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 71 */
#define LL_RAMCFG_WRP_PAGE_72  RAMCFG_WPR3_P72WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 72 */
#define LL_RAMCFG_WRP_PAGE_73  RAMCFG_WPR3_P73WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 73 */
#define LL_RAMCFG_WRP_PAGE_74  RAMCFG_WPR3_P74WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 74 */
#define LL_RAMCFG_WRP_PAGE_75  RAMCFG_WPR3_P75WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 75 */
#define LL_RAMCFG_WRP_PAGE_76  RAMCFG_WPR3_P76WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 76 */
#define LL_RAMCFG_WRP_PAGE_77  RAMCFG_WPR3_P77WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 77 */
#define LL_RAMCFG_WRP_PAGE_78  RAMCFG_WPR3_P78WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 78 */
#define LL_RAMCFG_WRP_PAGE_79  RAMCFG_WPR3_P79WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 79 */
#define LL_RAMCFG_WRP_PAGE_80  RAMCFG_WPR3_P80WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 80 */
#define LL_RAMCFG_WRP_PAGE_81  RAMCFG_WPR3_P81WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 81 */
#define LL_RAMCFG_WRP_PAGE_82  RAMCFG_WPR3_P82WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 82 */
#define LL_RAMCFG_WRP_PAGE_83  RAMCFG_WPR3_P83WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 83 */
#define LL_RAMCFG_WRP_PAGE_84  RAMCFG_WPR3_P84WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 84 */
#define LL_RAMCFG_WRP_PAGE_85  RAMCFG_WPR3_P85WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 85 */
#define LL_RAMCFG_WRP_PAGE_86  RAMCFG_WPR3_P86WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 86 */
#define LL_RAMCFG_WRP_PAGE_87  RAMCFG_WPR3_P87WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 87 */
#define LL_RAMCFG_WRP_PAGE_88  RAMCFG_WPR3_P88WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 88 */
#define LL_RAMCFG_WRP_PAGE_89  RAMCFG_WPR3_P89WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 89 */
#define LL_RAMCFG_WRP_PAGE_90  RAMCFG_WPR3_P90WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 90 */
#define LL_RAMCFG_WRP_PAGE_91  RAMCFG_WPR3_P91WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 91 */
#define LL_RAMCFG_WRP_PAGE_92  RAMCFG_WPR3_P92WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 92 */
#define LL_RAMCFG_WRP_PAGE_93  RAMCFG_WPR3_P93WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 93 */
#define LL_RAMCFG_WRP_PAGE_94  RAMCFG_WPR3_P94WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 94 */
#define LL_RAMCFG_WRP_PAGE_95  RAMCFG_WPR3_P95WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 95 */
#endif /* RAMCFG_WPR3_P64WP */
#if defined(RAMCFG_WPR4_P96WP)
#define LL_RAMCFG_WRP_PAGE_96  RAMCFG_WPR4_P96WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 96 */
#define LL_RAMCFG_WRP_PAGE_97  RAMCFG_WPR4_P97WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 97 */
#define LL_RAMCFG_WRP_PAGE_98  RAMCFG_WPR4_P98WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 98 */
#define LL_RAMCFG_WRP_PAGE_99  RAMCFG_WPR4_P99WP  /*!< LL RAMCFG WRITE PROTECTION PAGE 99 */
#define LL_RAMCFG_WRP_PAGE_100 RAMCFG_WPR4_P100WP /*!< LL RAMCFG WRITE PROTECTION PAGE 100 */
#define LL_RAMCFG_WRP_PAGE_101 RAMCFG_WPR4_P101WP /*!< LL RAMCFG WRITE PROTECTION PAGE 101 */
#define LL_RAMCFG_WRP_PAGE_102 RAMCFG_WPR4_P102WP /*!< LL RAMCFG WRITE PROTECTION PAGE 102 */
#define LL_RAMCFG_WRP_PAGE_103 RAMCFG_WPR4_P103WP /*!< LL RAMCFG WRITE PROTECTION PAGE 103 */
#define LL_RAMCFG_WRP_PAGE_104 RAMCFG_WPR4_P104WP /*!< LL RAMCFG WRITE PROTECTION PAGE 104 */
#define LL_RAMCFG_WRP_PAGE_105 RAMCFG_WPR4_P105WP /*!< LL RAMCFG WRITE PROTECTION PAGE 105 */
#define LL_RAMCFG_WRP_PAGE_106 RAMCFG_WPR4_P106WP /*!< LL RAMCFG WRITE PROTECTION PAGE 106 */
#define LL_RAMCFG_WRP_PAGE_107 RAMCFG_WPR4_P107WP /*!< LL RAMCFG WRITE PROTECTION PAGE 107 */
#define LL_RAMCFG_WRP_PAGE_108 RAMCFG_WPR4_P108WP /*!< LL RAMCFG WRITE PROTECTION PAGE 108 */
#define LL_RAMCFG_WRP_PAGE_109 RAMCFG_WPR4_P109WP /*!< LL RAMCFG WRITE PROTECTION PAGE 109 */
#define LL_RAMCFG_WRP_PAGE_110 RAMCFG_WPR4_P110WP /*!< LL RAMCFG WRITE PROTECTION PAGE 110 */
#define LL_RAMCFG_WRP_PAGE_111 RAMCFG_WPR4_P111WP /*!< LL RAMCFG WRITE PROTECTION PAGE 111 */
#define LL_RAMCFG_WRP_PAGE_112 RAMCFG_WPR4_P112WP /*!< LL RAMCFG WRITE PROTECTION PAGE 112 */
#define LL_RAMCFG_WRP_PAGE_113 RAMCFG_WPR4_P113WP /*!< LL RAMCFG WRITE PROTECTION PAGE 113 */
#define LL_RAMCFG_WRP_PAGE_114 RAMCFG_WPR4_P114WP /*!< LL RAMCFG WRITE PROTECTION PAGE 114 */
#define LL_RAMCFG_WRP_PAGE_115 RAMCFG_WPR4_P115WP /*!< LL RAMCFG WRITE PROTECTION PAGE 115 */
#define LL_RAMCFG_WRP_PAGE_116 RAMCFG_WPR4_P116WP /*!< LL RAMCFG WRITE PROTECTION PAGE 116 */
#define LL_RAMCFG_WRP_PAGE_117 RAMCFG_WPR4_P117WP /*!< LL RAMCFG WRITE PROTECTION PAGE 117 */
#define LL_RAMCFG_WRP_PAGE_118 RAMCFG_WPR4_P118WP /*!< LL RAMCFG WRITE PROTECTION PAGE 118 */
#define LL_RAMCFG_WRP_PAGE_119 RAMCFG_WPR4_P119WP /*!< LL RAMCFG WRITE PROTECTION PAGE 119 */
#define LL_RAMCFG_WRP_PAGE_120 RAMCFG_WPR4_P120WP /*!< LL RAMCFG WRITE PROTECTION PAGE 120 */
#define LL_RAMCFG_WRP_PAGE_121 RAMCFG_WPR4_P121WP /*!< LL RAMCFG WRITE PROTECTION PAGE 121 */
#define LL_RAMCFG_WRP_PAGE_122 RAMCFG_WPR4_P122WP /*!< LL RAMCFG WRITE PROTECTION PAGE 122 */
#define LL_RAMCFG_WRP_PAGE_123 RAMCFG_WPR4_P123WP /*!< LL RAMCFG WRITE PROTECTION PAGE 123 */
#define LL_RAMCFG_WRP_PAGE_124 RAMCFG_WPR4_P124WP /*!< LL RAMCFG WRITE PROTECTION PAGE 124 */
#define LL_RAMCFG_WRP_PAGE_125 RAMCFG_WPR4_P125WP /*!< LL RAMCFG WRITE PROTECTION PAGE 125 */
#define LL_RAMCFG_WRP_PAGE_126 RAMCFG_WPR4_P126WP /*!< LL RAMCFG WRITE PROTECTION PAGE 126 */
#define LL_RAMCFG_WRP_PAGE_127 RAMCFG_WPR4_P127WP /*!< LL RAMCFG WRITE PROTECTION PAGE 127 */
#endif /* RAMCFG_WPR4_P96WP */
/**
  * @}
  */

/** @defgroup RAMCFG_Erase_Keys RAMCFG Erase Keys
  * @{
  */
#define LL_RAMCFG_ERASE_KEY_1 (0xCAU) /*!< RAMCFG launch erase key 1 */
#define LL_RAMCFG_ERASE_KEY_2 (0x53U) /*!< RAMCFG launch erase key 2 */
/**
  * @}
  */

/** @defgroup RAMCFG_ECC_Keys RAMCFG ECC Keys
  * @{
  */
#define LL_RAMCFG_ECC_KEY_1   (0xAEU) /*!< RAMCFG launch ECC key 1   */
#define LL_RAMCFG_ECC_KEY_2   (0x75U) /*!< RAMCFG launch ECC key 2   */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup RAMCFG_LL_Exported_Macros LL RAMCFG Macros
  * @{
  */

/** @defgroup RAMCFG_LL_EM_COMMON_READ_REGISTERS Common Read Registers macro
  * @{
  */

/**
  * @brief  Write a value in RAMCFG register.
  * @param  instance RAMCFG Instance
  * @param  reg   Register to be written
  * @param  value Value to be written in the register
  */
#define LL_RAMCFG_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in RAMCFG register.
  * @param  instance RAMCFG Instance.
  * @param  reg      Register to be read.
  * @return Register value.
  */
#define LL_RAMCFG_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup RAMCFG_LL_Exported_Functions LL RAMCFG Functions
  * @{
  */

/**
  * @brief  Enable the RAMCFG ECC mechanism.
  * @rmtoll
  *  CR           ECCE           LL_RAMCFG_EnableECC \n
  *  CR           ALE            LL_RAMCFG_EnableECC
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_EnableECC(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_SET_BIT(p_ramcfg->CR, (RAMCFG_CR_ECCE | RAMCFG_CR_ALE));
}

/**
  * @brief  Disable the RAMCFG ECC mechanism.
  * @rmtoll
  *  CR           ECCE           LL_RAMCFG_DisableECC \n
  *  CR           ALE            LL_RAMCFG_DisableECC
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_DisableECC(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_CLEAR_BIT(p_ramcfg->CR, (RAMCFG_CR_ECCE | RAMCFG_CR_ALE));
}

/**
  * @brief  Check whether the RAMCFG ECC mechanism is enabled.
  * @rmtoll
  *  CR           ECCE           LL_RAMCFG_IsEnabledECC
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of ECC mechanism (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledECC(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->CR, RAMCFG_CR_ECCE) == (RAMCFG_CR_ECCE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the RAMCFG instance address latching error.
  * @rmtoll
  *  CR           ALE            LL_RAMCFG_EnableAddressLatch
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_EnableAddressLatch(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_SET_BIT(p_ramcfg->CR, RAMCFG_CR_ALE);
}

/**
  * @brief  Disable the RAMCFG instance address latching error.
  * @rmtoll
  *  CR           ALE            LL_RAMCFG_DisableAddressLatch
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_DisableAddressLatch(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_CLEAR_BIT(p_ramcfg->CR, RAMCFG_CR_ALE);
}

/**
  * @brief  Check whether the RAMCFG address latching error is enabled.
  * @rmtoll
  *  CR           ALE            LL_RAMCFG_IsEnabledAddressLatch
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of address latching error (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledAddressLatch(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->CR, RAMCFG_CR_ALE) == (RAMCFG_CR_ALE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable RAMCFG Erase operation.
  * @rmtoll
  *  CR           SRAMER         LL_RAMCFG_EnableSRAMErase
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_EnableSRAMErase(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_SET_BIT(p_ramcfg->CR, RAMCFG_CR_SRAMER);
}

/**
  * @brief  Check whether the RAMCFG erase operation is enabled.
  * @rmtoll
  *  CR           SRAMER         LL_RAMCFG_IsEnabledSRAMErase
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of erase operation (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledSRAMErase(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->CR, RAMCFG_CR_SRAMER) == (RAMCFG_CR_SRAMER)) ? 1UL : 0UL);
}

/**
  * @brief  Get the RAMCFG single error failing address.
  * @rmtoll
  *  SEAR           SEAR           LL_RAMCFG_GetECCSingleErrorAddress
  * @param  p_ramcfg RAMCFG Instance.
  * @return Single error address offset.
  */
__STATIC_INLINE uint32_t LL_RAMCFG_GetECCSingleErrorAddress(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((uint32_t)STM32_READ_REG(p_ramcfg->SEAR));
}

/**
  * @brief  Get the RAMCFG double error failing address.
  * @rmtoll
  *  DEAR           DEAR           LL_RAMCFG_GetECCDoubleErrorAddress
  * @param  p_ramcfg RAMCFG Instance.
  * @return Double error address offset.
  */
__STATIC_INLINE uint32_t LL_RAMCFG_GetECCDoubleErrorAddress(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((uint32_t)STM32_READ_REG(p_ramcfg->DEAR));
}

/**
  * @brief  Enable the ramcfg write protection for the given page.
  * @rmtoll
  *  WPR1           WPR1           LL_RAMCFG_EnablePageWRP_0_31
  * @param  p_ramcfg RAMCFG instance
  * @param  page_msk
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_0
  *         @arg @ref LL_RAMCFG_WRP_PAGE_1
  *         @arg @ref LL_RAMCFG_WRP_PAGE_2
  *         @arg @ref LL_RAMCFG_WRP_PAGE_3
  *         @arg @ref LL_RAMCFG_WRP_PAGE_4
  *         @arg @ref LL_RAMCFG_WRP_PAGE_5
  *         @arg @ref LL_RAMCFG_WRP_PAGE_6
  *         @arg @ref LL_RAMCFG_WRP_PAGE_7
  *         @arg @ref LL_RAMCFG_WRP_PAGE_8
  *         @arg @ref LL_RAMCFG_WRP_PAGE_9
  *         @arg @ref LL_RAMCFG_WRP_PAGE_10
  *         @arg @ref LL_RAMCFG_WRP_PAGE_11
  *         @arg @ref LL_RAMCFG_WRP_PAGE_12
  *         @arg @ref LL_RAMCFG_WRP_PAGE_13
  *         @arg @ref LL_RAMCFG_WRP_PAGE_14
  *         @arg @ref LL_RAMCFG_WRP_PAGE_15
  *         @arg @ref LL_RAMCFG_WRP_PAGE_16
  *         @arg @ref LL_RAMCFG_WRP_PAGE_17
  *         @arg @ref LL_RAMCFG_WRP_PAGE_18
  *         @arg @ref LL_RAMCFG_WRP_PAGE_19
  *         @arg @ref LL_RAMCFG_WRP_PAGE_20
  *         @arg @ref LL_RAMCFG_WRP_PAGE_21
  *         @arg @ref LL_RAMCFG_WRP_PAGE_22
  *         @arg @ref LL_RAMCFG_WRP_PAGE_23
  *         @arg @ref LL_RAMCFG_WRP_PAGE_24
  *         @arg @ref LL_RAMCFG_WRP_PAGE_25
  *         @arg @ref LL_RAMCFG_WRP_PAGE_26
  *         @arg @ref LL_RAMCFG_WRP_PAGE_27
  *         @arg @ref LL_RAMCFG_WRP_PAGE_28
  *         @arg @ref LL_RAMCFG_WRP_PAGE_29
  *         @arg @ref LL_RAMCFG_WRP_PAGE_30
  *         @arg @ref LL_RAMCFG_WRP_PAGE_31
  */
__STATIC_INLINE void LL_RAMCFG_EnablePageWRP_0_31(RAMCFG_TypeDef *p_ramcfg, uint32_t page_msk)
{
  STM32_SET_BIT(p_ramcfg->WPR1, page_msk);
}

/**
  * @brief  Check the ramcfg write protection state for the given page.
  * @rmtoll
  *  WPR1           WPR1           LL_RAMCFG_IsEnabledPageWRP_0_31
  * @param  p_ramcfg RAMCFG instance
  * @param  page
  *         This parameter can be one of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_0
  *         @arg @ref LL_RAMCFG_WRP_PAGE_1
  *         @arg @ref LL_RAMCFG_WRP_PAGE_2
  *         @arg @ref LL_RAMCFG_WRP_PAGE_3
  *         @arg @ref LL_RAMCFG_WRP_PAGE_4
  *         @arg @ref LL_RAMCFG_WRP_PAGE_5
  *         @arg @ref LL_RAMCFG_WRP_PAGE_6
  *         @arg @ref LL_RAMCFG_WRP_PAGE_7
  *         @arg @ref LL_RAMCFG_WRP_PAGE_8
  *         @arg @ref LL_RAMCFG_WRP_PAGE_9
  *         @arg @ref LL_RAMCFG_WRP_PAGE_10
  *         @arg @ref LL_RAMCFG_WRP_PAGE_11
  *         @arg @ref LL_RAMCFG_WRP_PAGE_12
  *         @arg @ref LL_RAMCFG_WRP_PAGE_13
  *         @arg @ref LL_RAMCFG_WRP_PAGE_14
  *         @arg @ref LL_RAMCFG_WRP_PAGE_15
  *         @arg @ref LL_RAMCFG_WRP_PAGE_16
  *         @arg @ref LL_RAMCFG_WRP_PAGE_17
  *         @arg @ref LL_RAMCFG_WRP_PAGE_18
  *         @arg @ref LL_RAMCFG_WRP_PAGE_19
  *         @arg @ref LL_RAMCFG_WRP_PAGE_20
  *         @arg @ref LL_RAMCFG_WRP_PAGE_21
  *         @arg @ref LL_RAMCFG_WRP_PAGE_22
  *         @arg @ref LL_RAMCFG_WRP_PAGE_23
  *         @arg @ref LL_RAMCFG_WRP_PAGE_24
  *         @arg @ref LL_RAMCFG_WRP_PAGE_25
  *         @arg @ref LL_RAMCFG_WRP_PAGE_26
  *         @arg @ref LL_RAMCFG_WRP_PAGE_27
  *         @arg @ref LL_RAMCFG_WRP_PAGE_28
  *         @arg @ref LL_RAMCFG_WRP_PAGE_29
  *         @arg @ref LL_RAMCFG_WRP_PAGE_30
  *         @arg @ref LL_RAMCFG_WRP_PAGE_31
  * @return State of the selected page (1 : write protected, 0 not write protected)
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledPageWRP_0_31(const RAMCFG_TypeDef *p_ramcfg, uint32_t page)
{
  return ((((uint32_t)STM32_READ_BIT(p_ramcfg->WPR1, page)) == page) ? 1U : 0U);
}

/**
  * @brief  Get the ramcfg write protection for the given page.
  * @rmtoll
  *  WPR1           WPR1           LL_RAMCFG_GetPageWRP_0_31
  * @param  p_ramcfg RAMCFG instance
  * @param  page_msk
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_0
  *         @arg @ref LL_RAMCFG_WRP_PAGE_1
  *         @arg @ref LL_RAMCFG_WRP_PAGE_2
  *         @arg @ref LL_RAMCFG_WRP_PAGE_3
  *         @arg @ref LL_RAMCFG_WRP_PAGE_4
  *         @arg @ref LL_RAMCFG_WRP_PAGE_5
  *         @arg @ref LL_RAMCFG_WRP_PAGE_6
  *         @arg @ref LL_RAMCFG_WRP_PAGE_7
  *         @arg @ref LL_RAMCFG_WRP_PAGE_8
  *         @arg @ref LL_RAMCFG_WRP_PAGE_9
  *         @arg @ref LL_RAMCFG_WRP_PAGE_10
  *         @arg @ref LL_RAMCFG_WRP_PAGE_11
  *         @arg @ref LL_RAMCFG_WRP_PAGE_12
  *         @arg @ref LL_RAMCFG_WRP_PAGE_13
  *         @arg @ref LL_RAMCFG_WRP_PAGE_14
  *         @arg @ref LL_RAMCFG_WRP_PAGE_15
  *         @arg @ref LL_RAMCFG_WRP_PAGE_16
  *         @arg @ref LL_RAMCFG_WRP_PAGE_17
  *         @arg @ref LL_RAMCFG_WRP_PAGE_18
  *         @arg @ref LL_RAMCFG_WRP_PAGE_19
  *         @arg @ref LL_RAMCFG_WRP_PAGE_20
  *         @arg @ref LL_RAMCFG_WRP_PAGE_21
  *         @arg @ref LL_RAMCFG_WRP_PAGE_22
  *         @arg @ref LL_RAMCFG_WRP_PAGE_23
  *         @arg @ref LL_RAMCFG_WRP_PAGE_24
  *         @arg @ref LL_RAMCFG_WRP_PAGE_25
  *         @arg @ref LL_RAMCFG_WRP_PAGE_26
  *         @arg @ref LL_RAMCFG_WRP_PAGE_27
  *         @arg @ref LL_RAMCFG_WRP_PAGE_28
  *         @arg @ref LL_RAMCFG_WRP_PAGE_29
  *         @arg @ref LL_RAMCFG_WRP_PAGE_30
  *         @arg @ref LL_RAMCFG_WRP_PAGE_31
  * @return The return value can be a 0x00000000U or combination of the following values :
  *         @arg @ref LL_RAMCFG_WRP_PAGE_0
  *         @arg @ref LL_RAMCFG_WRP_PAGE_1
  *         @arg @ref LL_RAMCFG_WRP_PAGE_2
  *         @arg @ref LL_RAMCFG_WRP_PAGE_3
  *         @arg @ref LL_RAMCFG_WRP_PAGE_4
  *         @arg @ref LL_RAMCFG_WRP_PAGE_5
  *         @arg @ref LL_RAMCFG_WRP_PAGE_6
  *         @arg @ref LL_RAMCFG_WRP_PAGE_7
  *         @arg @ref LL_RAMCFG_WRP_PAGE_8
  *         @arg @ref LL_RAMCFG_WRP_PAGE_9
  *         @arg @ref LL_RAMCFG_WRP_PAGE_10
  *         @arg @ref LL_RAMCFG_WRP_PAGE_11
  *         @arg @ref LL_RAMCFG_WRP_PAGE_12
  *         @arg @ref LL_RAMCFG_WRP_PAGE_13
  *         @arg @ref LL_RAMCFG_WRP_PAGE_14
  *         @arg @ref LL_RAMCFG_WRP_PAGE_15
  *         @arg @ref LL_RAMCFG_WRP_PAGE_16
  *         @arg @ref LL_RAMCFG_WRP_PAGE_17
  *         @arg @ref LL_RAMCFG_WRP_PAGE_18
  *         @arg @ref LL_RAMCFG_WRP_PAGE_19
  *         @arg @ref LL_RAMCFG_WRP_PAGE_20
  *         @arg @ref LL_RAMCFG_WRP_PAGE_21
  *         @arg @ref LL_RAMCFG_WRP_PAGE_22
  *         @arg @ref LL_RAMCFG_WRP_PAGE_23
  *         @arg @ref LL_RAMCFG_WRP_PAGE_24
  *         @arg @ref LL_RAMCFG_WRP_PAGE_25
  *         @arg @ref LL_RAMCFG_WRP_PAGE_26
  *         @arg @ref LL_RAMCFG_WRP_PAGE_27
  *         @arg @ref LL_RAMCFG_WRP_PAGE_28
  *         @arg @ref LL_RAMCFG_WRP_PAGE_29
  *         @arg @ref LL_RAMCFG_WRP_PAGE_30
  *         @arg @ref LL_RAMCFG_WRP_PAGE_31
  */
__STATIC_INLINE uint32_t LL_RAMCFG_GetPageWRP_0_31(const RAMCFG_TypeDef *p_ramcfg, uint32_t page_msk)
{
  return ((uint32_t)STM32_READ_BIT(p_ramcfg->WPR1, page_msk));
}

#if defined(LL_RAMCFG_WRP_PAGE_32)
/**
  * @brief  Enable the ramcfg write protection for the given page.
  * @rmtoll
  *  WPR2           WPR2           LL_RAMCFG_EnablePageWRP_32_63
  * @param  p_ramcfg RAMCFG instance
  * @param  page_msk
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_32
  *         @arg @ref LL_RAMCFG_WRP_PAGE_33
  *         @arg @ref LL_RAMCFG_WRP_PAGE_34
  *         @arg @ref LL_RAMCFG_WRP_PAGE_35
  *         @arg @ref LL_RAMCFG_WRP_PAGE_36
  *         @arg @ref LL_RAMCFG_WRP_PAGE_37
  *         @arg @ref LL_RAMCFG_WRP_PAGE_38
  *         @arg @ref LL_RAMCFG_WRP_PAGE_39
  *         @arg @ref LL_RAMCFG_WRP_PAGE_40
  *         @arg @ref LL_RAMCFG_WRP_PAGE_41
  *         @arg @ref LL_RAMCFG_WRP_PAGE_42
  *         @arg @ref LL_RAMCFG_WRP_PAGE_43
  *         @arg @ref LL_RAMCFG_WRP_PAGE_44
  *         @arg @ref LL_RAMCFG_WRP_PAGE_45
  *         @arg @ref LL_RAMCFG_WRP_PAGE_46
  *         @arg @ref LL_RAMCFG_WRP_PAGE_47
  *         @arg @ref LL_RAMCFG_WRP_PAGE_48
  *         @arg @ref LL_RAMCFG_WRP_PAGE_49
  *         @arg @ref LL_RAMCFG_WRP_PAGE_50
  *         @arg @ref LL_RAMCFG_WRP_PAGE_51
  *         @arg @ref LL_RAMCFG_WRP_PAGE_52
  *         @arg @ref LL_RAMCFG_WRP_PAGE_53
  *         @arg @ref LL_RAMCFG_WRP_PAGE_54
  *         @arg @ref LL_RAMCFG_WRP_PAGE_55
  *         @arg @ref LL_RAMCFG_WRP_PAGE_56
  *         @arg @ref LL_RAMCFG_WRP_PAGE_57
  *         @arg @ref LL_RAMCFG_WRP_PAGE_58
  *         @arg @ref LL_RAMCFG_WRP_PAGE_59
  *         @arg @ref LL_RAMCFG_WRP_PAGE_60
  *         @arg @ref LL_RAMCFG_WRP_PAGE_61
  *         @arg @ref LL_RAMCFG_WRP_PAGE_62
  *         @arg @ref LL_RAMCFG_WRP_PAGE_63
  */
__STATIC_INLINE void LL_RAMCFG_EnablePageWRP_32_63(RAMCFG_TypeDef *p_ramcfg, uint32_t page_msk)
{
  STM32_SET_BIT(p_ramcfg->WPR2, page_msk);
}

/**
  * @brief  Check the ramcfg write protection state for the given page.
  * @rmtoll
  *  WPR2           WPR2           LL_RAMCFG_IsEnabledPageWRP_32_63
  * @param  p_ramcfg RAMCFG instance
  * @param  page
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_32
  *         @arg @ref LL_RAMCFG_WRP_PAGE_33
  *         @arg @ref LL_RAMCFG_WRP_PAGE_34
  *         @arg @ref LL_RAMCFG_WRP_PAGE_35
  *         @arg @ref LL_RAMCFG_WRP_PAGE_36
  *         @arg @ref LL_RAMCFG_WRP_PAGE_37
  *         @arg @ref LL_RAMCFG_WRP_PAGE_38
  *         @arg @ref LL_RAMCFG_WRP_PAGE_39
  *         @arg @ref LL_RAMCFG_WRP_PAGE_40
  *         @arg @ref LL_RAMCFG_WRP_PAGE_41
  *         @arg @ref LL_RAMCFG_WRP_PAGE_42
  *         @arg @ref LL_RAMCFG_WRP_PAGE_43
  *         @arg @ref LL_RAMCFG_WRP_PAGE_44
  *         @arg @ref LL_RAMCFG_WRP_PAGE_45
  *         @arg @ref LL_RAMCFG_WRP_PAGE_46
  *         @arg @ref LL_RAMCFG_WRP_PAGE_47
  *         @arg @ref LL_RAMCFG_WRP_PAGE_48
  *         @arg @ref LL_RAMCFG_WRP_PAGE_49
  *         @arg @ref LL_RAMCFG_WRP_PAGE_50
  *         @arg @ref LL_RAMCFG_WRP_PAGE_51
  *         @arg @ref LL_RAMCFG_WRP_PAGE_52
  *         @arg @ref LL_RAMCFG_WRP_PAGE_53
  *         @arg @ref LL_RAMCFG_WRP_PAGE_54
  *         @arg @ref LL_RAMCFG_WRP_PAGE_55
  *         @arg @ref LL_RAMCFG_WRP_PAGE_56
  *         @arg @ref LL_RAMCFG_WRP_PAGE_57
  *         @arg @ref LL_RAMCFG_WRP_PAGE_58
  *         @arg @ref LL_RAMCFG_WRP_PAGE_59
  *         @arg @ref LL_RAMCFG_WRP_PAGE_60
  *         @arg @ref LL_RAMCFG_WRP_PAGE_61
  *         @arg @ref LL_RAMCFG_WRP_PAGE_62
  *         @arg @ref LL_RAMCFG_WRP_PAGE_63
  * @return State of the selected page (1 : write protected, 0 not write protected)
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledPageWRP_32_63(const RAMCFG_TypeDef *p_ramcfg, uint32_t page)
{
  return ((((uint32_t)STM32_READ_BIT(p_ramcfg->WPR2, page)) == page) ? 1U : 0U);
}

/**
  * @brief  Get the ramcfg write protection for the given page.
  * @rmtoll
  *  WPR2           WPR2           LL_RAMCFG_GetPageWRP_32_63
  * @param  p_ramcfg RAMCFG instance
  * @param  page_msk
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_32
  *         @arg @ref LL_RAMCFG_WRP_PAGE_33
  *         @arg @ref LL_RAMCFG_WRP_PAGE_34
  *         @arg @ref LL_RAMCFG_WRP_PAGE_35
  *         @arg @ref LL_RAMCFG_WRP_PAGE_36
  *         @arg @ref LL_RAMCFG_WRP_PAGE_37
  *         @arg @ref LL_RAMCFG_WRP_PAGE_38
  *         @arg @ref LL_RAMCFG_WRP_PAGE_39
  *         @arg @ref LL_RAMCFG_WRP_PAGE_40
  *         @arg @ref LL_RAMCFG_WRP_PAGE_41
  *         @arg @ref LL_RAMCFG_WRP_PAGE_42
  *         @arg @ref LL_RAMCFG_WRP_PAGE_43
  *         @arg @ref LL_RAMCFG_WRP_PAGE_44
  *         @arg @ref LL_RAMCFG_WRP_PAGE_45
  *         @arg @ref LL_RAMCFG_WRP_PAGE_46
  *         @arg @ref LL_RAMCFG_WRP_PAGE_47
  *         @arg @ref LL_RAMCFG_WRP_PAGE_48
  *         @arg @ref LL_RAMCFG_WRP_PAGE_49
  *         @arg @ref LL_RAMCFG_WRP_PAGE_50
  *         @arg @ref LL_RAMCFG_WRP_PAGE_51
  *         @arg @ref LL_RAMCFG_WRP_PAGE_52
  *         @arg @ref LL_RAMCFG_WRP_PAGE_53
  *         @arg @ref LL_RAMCFG_WRP_PAGE_54
  *         @arg @ref LL_RAMCFG_WRP_PAGE_55
  *         @arg @ref LL_RAMCFG_WRP_PAGE_56
  *         @arg @ref LL_RAMCFG_WRP_PAGE_57
  *         @arg @ref LL_RAMCFG_WRP_PAGE_58
  *         @arg @ref LL_RAMCFG_WRP_PAGE_59
  *         @arg @ref LL_RAMCFG_WRP_PAGE_60
  *         @arg @ref LL_RAMCFG_WRP_PAGE_61
  *         @arg @ref LL_RAMCFG_WRP_PAGE_62
  *         @arg @ref LL_RAMCFG_WRP_PAGE_63
  * @return The return value can be a 0x00000000U or combination of the following values :
  *         @arg @ref LL_RAMCFG_WRP_PAGE_32
  *         @arg @ref LL_RAMCFG_WRP_PAGE_33
  *         @arg @ref LL_RAMCFG_WRP_PAGE_34
  *         @arg @ref LL_RAMCFG_WRP_PAGE_35
  *         @arg @ref LL_RAMCFG_WRP_PAGE_36
  *         @arg @ref LL_RAMCFG_WRP_PAGE_37
  *         @arg @ref LL_RAMCFG_WRP_PAGE_38
  *         @arg @ref LL_RAMCFG_WRP_PAGE_39
  *         @arg @ref LL_RAMCFG_WRP_PAGE_40
  *         @arg @ref LL_RAMCFG_WRP_PAGE_41
  *         @arg @ref LL_RAMCFG_WRP_PAGE_42
  *         @arg @ref LL_RAMCFG_WRP_PAGE_43
  *         @arg @ref LL_RAMCFG_WRP_PAGE_44
  *         @arg @ref LL_RAMCFG_WRP_PAGE_45
  *         @arg @ref LL_RAMCFG_WRP_PAGE_46
  *         @arg @ref LL_RAMCFG_WRP_PAGE_47
  *         @arg @ref LL_RAMCFG_WRP_PAGE_48
  *         @arg @ref LL_RAMCFG_WRP_PAGE_49
  *         @arg @ref LL_RAMCFG_WRP_PAGE_50
  *         @arg @ref LL_RAMCFG_WRP_PAGE_51
  *         @arg @ref LL_RAMCFG_WRP_PAGE_52
  *         @arg @ref LL_RAMCFG_WRP_PAGE_53
  *         @arg @ref LL_RAMCFG_WRP_PAGE_54
  *         @arg @ref LL_RAMCFG_WRP_PAGE_55
  *         @arg @ref LL_RAMCFG_WRP_PAGE_56
  *         @arg @ref LL_RAMCFG_WRP_PAGE_57
  *         @arg @ref LL_RAMCFG_WRP_PAGE_58
  *         @arg @ref LL_RAMCFG_WRP_PAGE_59
  *         @arg @ref LL_RAMCFG_WRP_PAGE_60
  *         @arg @ref LL_RAMCFG_WRP_PAGE_61
  *         @arg @ref LL_RAMCFG_WRP_PAGE_62
  *         @arg @ref LL_RAMCFG_WRP_PAGE_63
  */
__STATIC_INLINE uint32_t LL_RAMCFG_GetPageWRP_32_63(const RAMCFG_TypeDef *p_ramcfg, uint32_t page_msk)
{
  return ((uint32_t)STM32_READ_BIT(p_ramcfg->WPR2, page_msk));
}
#endif /* LL_RAMCFG_WRP_PAGE_32 */

#if defined (LL_RAMCFG_WRP_PAGE_64)
/**
  * @brief  Enable the ramcfg write protection for the given page.
  * @rmtoll
  *  WPR3           WPR3           LL_RAMCFG_EnablePageWRP_64_95
  * @param  p_ramcfg RAMCFG instance
  * @param  page_msk
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_64
  *         @arg @ref LL_RAMCFG_WRP_PAGE_65
  *         @arg @ref LL_RAMCFG_WRP_PAGE_66
  *         @arg @ref LL_RAMCFG_WRP_PAGE_67
  *         @arg @ref LL_RAMCFG_WRP_PAGE_68
  *         @arg @ref LL_RAMCFG_WRP_PAGE_69
  *         @arg @ref LL_RAMCFG_WRP_PAGE_70
  *         @arg @ref LL_RAMCFG_WRP_PAGE_71
  *         @arg @ref LL_RAMCFG_WRP_PAGE_72
  *         @arg @ref LL_RAMCFG_WRP_PAGE_73
  *         @arg @ref LL_RAMCFG_WRP_PAGE_74
  *         @arg @ref LL_RAMCFG_WRP_PAGE_75
  *         @arg @ref LL_RAMCFG_WRP_PAGE_76
  *         @arg @ref LL_RAMCFG_WRP_PAGE_77
  *         @arg @ref LL_RAMCFG_WRP_PAGE_78
  *         @arg @ref LL_RAMCFG_WRP_PAGE_79
  *         @arg @ref LL_RAMCFG_WRP_PAGE_80
  *         @arg @ref LL_RAMCFG_WRP_PAGE_81
  *         @arg @ref LL_RAMCFG_WRP_PAGE_82
  *         @arg @ref LL_RAMCFG_WRP_PAGE_83
  *         @arg @ref LL_RAMCFG_WRP_PAGE_84
  *         @arg @ref LL_RAMCFG_WRP_PAGE_85
  *         @arg @ref LL_RAMCFG_WRP_PAGE_86
  *         @arg @ref LL_RAMCFG_WRP_PAGE_87
  *         @arg @ref LL_RAMCFG_WRP_PAGE_88
  *         @arg @ref LL_RAMCFG_WRP_PAGE_89
  *         @arg @ref LL_RAMCFG_WRP_PAGE_90
  *         @arg @ref LL_RAMCFG_WRP_PAGE_91
  *         @arg @ref LL_RAMCFG_WRP_PAGE_92
  *         @arg @ref LL_RAMCFG_WRP_PAGE_93
  *         @arg @ref LL_RAMCFG_WRP_PAGE_94
  *         @arg @ref LL_RAMCFG_WRP_PAGE_95
  */
__STATIC_INLINE void LL_RAMCFG_EnablePageWRP_64_95(RAMCFG_TypeDef *p_ramcfg, uint32_t page_msk)
{
  STM32_SET_BIT(p_ramcfg->WPR3, page_msk);
}

/**
  * @brief  Check the ramcfg write protection state for the given page.
  * @rmtoll
  *  WPR3           WPR3           LL_RAMCFG_IsEnabledPageWRP_64_95
  * @param  p_ramcfg RAMCFG instance
  * @param  page_
  *         This parameter can be one of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_64
  *         @arg @ref LL_RAMCFG_WRP_PAGE_65
  *         @arg @ref LL_RAMCFG_WRP_PAGE_66
  *         @arg @ref LL_RAMCFG_WRP_PAGE_67
  *         @arg @ref LL_RAMCFG_WRP_PAGE_68
  *         @arg @ref LL_RAMCFG_WRP_PAGE_69
  *         @arg @ref LL_RAMCFG_WRP_PAGE_70
  *         @arg @ref LL_RAMCFG_WRP_PAGE_71
  *         @arg @ref LL_RAMCFG_WRP_PAGE_72
  *         @arg @ref LL_RAMCFG_WRP_PAGE_73
  *         @arg @ref LL_RAMCFG_WRP_PAGE_74
  *         @arg @ref LL_RAMCFG_WRP_PAGE_75
  *         @arg @ref LL_RAMCFG_WRP_PAGE_76
  *         @arg @ref LL_RAMCFG_WRP_PAGE_77
  *         @arg @ref LL_RAMCFG_WRP_PAGE_78
  *         @arg @ref LL_RAMCFG_WRP_PAGE_79
  *         @arg @ref LL_RAMCFG_WRP_PAGE_80
  *         @arg @ref LL_RAMCFG_WRP_PAGE_81
  *         @arg @ref LL_RAMCFG_WRP_PAGE_82
  *         @arg @ref LL_RAMCFG_WRP_PAGE_83
  *         @arg @ref LL_RAMCFG_WRP_PAGE_84
  *         @arg @ref LL_RAMCFG_WRP_PAGE_85
  *         @arg @ref LL_RAMCFG_WRP_PAGE_86
  *         @arg @ref LL_RAMCFG_WRP_PAGE_87
  *         @arg @ref LL_RAMCFG_WRP_PAGE_88
  *         @arg @ref LL_RAMCFG_WRP_PAGE_89
  *         @arg @ref LL_RAMCFG_WRP_PAGE_90
  *         @arg @ref LL_RAMCFG_WRP_PAGE_91
  *         @arg @ref LL_RAMCFG_WRP_PAGE_92
  *         @arg @ref LL_RAMCFG_WRP_PAGE_93
  *         @arg @ref LL_RAMCFG_WRP_PAGE_94
  *         @arg @ref LL_RAMCFG_WRP_PAGE_95
  * @return State of the selected page (1: write protected, 0: not write protected)
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledPageWRP_64_95(const RAMCFG_TypeDef *p_ramcfg, uint32_t page)
{
  return ((((uint32_t)STM32_READ_BIT(p_ramcfg->WPR3, page)) == page) ? 1U : 0U);
}

/**
  * @brief  Get the ramcfg write protection for the given page.
  * @rmtoll
  *  WPR3           WPR3           LL_RAMCFG_GetPageWRP_64_95
  * @param  p_ramcfg RAMCFG instance
  * @param  page_msk
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_64
  *         @arg @ref LL_RAMCFG_WRP_PAGE_65
  *         @arg @ref LL_RAMCFG_WRP_PAGE_66
  *         @arg @ref LL_RAMCFG_WRP_PAGE_67
  *         @arg @ref LL_RAMCFG_WRP_PAGE_68
  *         @arg @ref LL_RAMCFG_WRP_PAGE_69
  *         @arg @ref LL_RAMCFG_WRP_PAGE_70
  *         @arg @ref LL_RAMCFG_WRP_PAGE_71
  *         @arg @ref LL_RAMCFG_WRP_PAGE_72
  *         @arg @ref LL_RAMCFG_WRP_PAGE_73
  *         @arg @ref LL_RAMCFG_WRP_PAGE_74
  *         @arg @ref LL_RAMCFG_WRP_PAGE_75
  *         @arg @ref LL_RAMCFG_WRP_PAGE_76
  *         @arg @ref LL_RAMCFG_WRP_PAGE_77
  *         @arg @ref LL_RAMCFG_WRP_PAGE_78
  *         @arg @ref LL_RAMCFG_WRP_PAGE_79
  *         @arg @ref LL_RAMCFG_WRP_PAGE_80
  *         @arg @ref LL_RAMCFG_WRP_PAGE_81
  *         @arg @ref LL_RAMCFG_WRP_PAGE_82
  *         @arg @ref LL_RAMCFG_WRP_PAGE_83
  *         @arg @ref LL_RAMCFG_WRP_PAGE_84
  *         @arg @ref LL_RAMCFG_WRP_PAGE_85
  *         @arg @ref LL_RAMCFG_WRP_PAGE_86
  *         @arg @ref LL_RAMCFG_WRP_PAGE_87
  *         @arg @ref LL_RAMCFG_WRP_PAGE_88
  *         @arg @ref LL_RAMCFG_WRP_PAGE_89
  *         @arg @ref LL_RAMCFG_WRP_PAGE_90
  *         @arg @ref LL_RAMCFG_WRP_PAGE_91
  *         @arg @ref LL_RAMCFG_WRP_PAGE_92
  *         @arg @ref LL_RAMCFG_WRP_PAGE_93
  *         @arg @ref LL_RAMCFG_WRP_PAGE_94
  *         @arg @ref LL_RAMCFG_WRP_PAGE_95
  * @return The return value can be a 0x00000000U or combination of the following values :
  *         @arg @ref LL_RAMCFG_WRP_PAGE_64
  *         @arg @ref LL_RAMCFG_WRP_PAGE_65
  *         @arg @ref LL_RAMCFG_WRP_PAGE_66
  *         @arg @ref LL_RAMCFG_WRP_PAGE_67
  *         @arg @ref LL_RAMCFG_WRP_PAGE_68
  *         @arg @ref LL_RAMCFG_WRP_PAGE_69
  *         @arg @ref LL_RAMCFG_WRP_PAGE_70
  *         @arg @ref LL_RAMCFG_WRP_PAGE_71
  *         @arg @ref LL_RAMCFG_WRP_PAGE_72
  *         @arg @ref LL_RAMCFG_WRP_PAGE_73
  *         @arg @ref LL_RAMCFG_WRP_PAGE_74
  *         @arg @ref LL_RAMCFG_WRP_PAGE_75
  *         @arg @ref LL_RAMCFG_WRP_PAGE_76
  *         @arg @ref LL_RAMCFG_WRP_PAGE_77
  *         @arg @ref LL_RAMCFG_WRP_PAGE_78
  *         @arg @ref LL_RAMCFG_WRP_PAGE_79
  *         @arg @ref LL_RAMCFG_WRP_PAGE_80
  *         @arg @ref LL_RAMCFG_WRP_PAGE_81
  *         @arg @ref LL_RAMCFG_WRP_PAGE_82
  *         @arg @ref LL_RAMCFG_WRP_PAGE_83
  *         @arg @ref LL_RAMCFG_WRP_PAGE_84
  *         @arg @ref LL_RAMCFG_WRP_PAGE_85
  *         @arg @ref LL_RAMCFG_WRP_PAGE_86
  *         @arg @ref LL_RAMCFG_WRP_PAGE_87
  *         @arg @ref LL_RAMCFG_WRP_PAGE_88
  *         @arg @ref LL_RAMCFG_WRP_PAGE_89
  *         @arg @ref LL_RAMCFG_WRP_PAGE_90
  *         @arg @ref LL_RAMCFG_WRP_PAGE_91
  *         @arg @ref LL_RAMCFG_WRP_PAGE_92
  *         @arg @ref LL_RAMCFG_WRP_PAGE_93
  *         @arg @ref LL_RAMCFG_WRP_PAGE_94
  *         @arg @ref LL_RAMCFG_WRP_PAGE_95
  */
__STATIC_INLINE uint32_t LL_RAMCFG_GetPageWRP_64_95(const RAMCFG_TypeDef *p_ramcfg, uint32_t page_msk)
{
  return ((uint32_t)STM32_READ_BIT(p_ramcfg->WPR3, page_msk));
}
#endif /* LL_RAMCFG_WRP_PAGE_64 */

#if defined (LL_RAMCFG_WRP_PAGE_96)
/**
  * @brief  Enable the ramcfg write protection for the given page.
  * @rmtoll
  *  WPR4           WPR4           LL_RAMCFG_EnablePageWRP_96_127
  * @param  p_ramcfg RAMCFG instance
  * @param  page_msk
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_96
  *         @arg @ref LL_RAMCFG_WRP_PAGE_97
  *         @arg @ref LL_RAMCFG_WRP_PAGE_98
  *         @arg @ref LL_RAMCFG_WRP_PAGE_99
  *         @arg @ref LL_RAMCFG_WRP_PAGE_100
  *         @arg @ref LL_RAMCFG_WRP_PAGE_101
  *         @arg @ref LL_RAMCFG_WRP_PAGE_102
  *         @arg @ref LL_RAMCFG_WRP_PAGE_103
  *         @arg @ref LL_RAMCFG_WRP_PAGE_104
  *         @arg @ref LL_RAMCFG_WRP_PAGE_105
  *         @arg @ref LL_RAMCFG_WRP_PAGE_106
  *         @arg @ref LL_RAMCFG_WRP_PAGE_107
  *         @arg @ref LL_RAMCFG_WRP_PAGE_108
  *         @arg @ref LL_RAMCFG_WRP_PAGE_109
  *         @arg @ref LL_RAMCFG_WRP_PAGE_110
  *         @arg @ref LL_RAMCFG_WRP_PAGE_111
  *         @arg @ref LL_RAMCFG_WRP_PAGE_112
  *         @arg @ref LL_RAMCFG_WRP_PAGE_113
  *         @arg @ref LL_RAMCFG_WRP_PAGE_114
  *         @arg @ref LL_RAMCFG_WRP_PAGE_115
  *         @arg @ref LL_RAMCFG_WRP_PAGE_116
  *         @arg @ref LL_RAMCFG_WRP_PAGE_117
  *         @arg @ref LL_RAMCFG_WRP_PAGE_118
  *         @arg @ref LL_RAMCFG_WRP_PAGE_119
  *         @arg @ref LL_RAMCFG_WRP_PAGE_120
  *         @arg @ref LL_RAMCFG_WRP_PAGE_121
  *         @arg @ref LL_RAMCFG_WRP_PAGE_122
  *         @arg @ref LL_RAMCFG_WRP_PAGE_123
  *         @arg @ref LL_RAMCFG_WRP_PAGE_124
  *         @arg @ref LL_RAMCFG_WRP_PAGE_125
  *         @arg @ref LL_RAMCFG_WRP_PAGE_126
  *         @arg @ref LL_RAMCFG_WRP_PAGE_127
  */
__STATIC_INLINE void LL_RAMCFG_EnablePageWRP_96_127(RAMCFG_TypeDef *p_ramcfg, uint32_t page_msk)
{
  STM32_SET_BIT(p_ramcfg->WPR4, page_msk);
}

/**
  * @brief  Check the ramcfg write protection state for the given page.
  * @rmtoll
  *  WPR4           WPR4           LL_RAMCFG_IsEnabledPageWRP_96_127
  * @param  p_ramcfg RAMCFG instance
  * @param  page
  *         This parameter can be one of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_96
  *         @arg @ref LL_RAMCFG_WRP_PAGE_97
  *         @arg @ref LL_RAMCFG_WRP_PAGE_98
  *         @arg @ref LL_RAMCFG_WRP_PAGE_99
  *         @arg @ref LL_RAMCFG_WRP_PAGE_100
  *         @arg @ref LL_RAMCFG_WRP_PAGE_101
  *         @arg @ref LL_RAMCFG_WRP_PAGE_102
  *         @arg @ref LL_RAMCFG_WRP_PAGE_103
  *         @arg @ref LL_RAMCFG_WRP_PAGE_104
  *         @arg @ref LL_RAMCFG_WRP_PAGE_105
  *         @arg @ref LL_RAMCFG_WRP_PAGE_106
  *         @arg @ref LL_RAMCFG_WRP_PAGE_107
  *         @arg @ref LL_RAMCFG_WRP_PAGE_108
  *         @arg @ref LL_RAMCFG_WRP_PAGE_109
  *         @arg @ref LL_RAMCFG_WRP_PAGE_110
  *         @arg @ref LL_RAMCFG_WRP_PAGE_111
  *         @arg @ref LL_RAMCFG_WRP_PAGE_112
  *         @arg @ref LL_RAMCFG_WRP_PAGE_113
  *         @arg @ref LL_RAMCFG_WRP_PAGE_114
  *         @arg @ref LL_RAMCFG_WRP_PAGE_115
  *         @arg @ref LL_RAMCFG_WRP_PAGE_116
  *         @arg @ref LL_RAMCFG_WRP_PAGE_117
  *         @arg @ref LL_RAMCFG_WRP_PAGE_118
  *         @arg @ref LL_RAMCFG_WRP_PAGE_119
  *         @arg @ref LL_RAMCFG_WRP_PAGE_120
  *         @arg @ref LL_RAMCFG_WRP_PAGE_121
  *         @arg @ref LL_RAMCFG_WRP_PAGE_122
  *         @arg @ref LL_RAMCFG_WRP_PAGE_123
  *         @arg @ref LL_RAMCFG_WRP_PAGE_124
  *         @arg @ref LL_RAMCFG_WRP_PAGE_125
  *         @arg @ref LL_RAMCFG_WRP_PAGE_126
  *         @arg @ref LL_RAMCFG_WRP_PAGE_127
  * @return State of the selected page (1 : write protected, 0 not write protected)
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledPageWRP_96_127(const RAMCFG_TypeDef *p_ramcfg, uint32_t page)
{
  return ((((uint32_t)STM32_READ_BIT(p_ramcfg->WPR4, page)) == page) ? 1U : 0U);
}

/**
  * @brief  Get the ramcfg write protection for the given page.
  * @rmtoll
  *  WPR4           WPR4           LL_RAMCFG_GetPageWRP_96_127
  * @param  p_ramcfg RAMCFG instance
  * @param  page_msk
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_RAMCFG_WRP_PAGE_96
  *         @arg @ref LL_RAMCFG_WRP_PAGE_97
  *         @arg @ref LL_RAMCFG_WRP_PAGE_98
  *         @arg @ref LL_RAMCFG_WRP_PAGE_99
  *         @arg @ref LL_RAMCFG_WRP_PAGE_100
  *         @arg @ref LL_RAMCFG_WRP_PAGE_101
  *         @arg @ref LL_RAMCFG_WRP_PAGE_102
  *         @arg @ref LL_RAMCFG_WRP_PAGE_103
  *         @arg @ref LL_RAMCFG_WRP_PAGE_104
  *         @arg @ref LL_RAMCFG_WRP_PAGE_105
  *         @arg @ref LL_RAMCFG_WRP_PAGE_106
  *         @arg @ref LL_RAMCFG_WRP_PAGE_107
  *         @arg @ref LL_RAMCFG_WRP_PAGE_108
  *         @arg @ref LL_RAMCFG_WRP_PAGE_109
  *         @arg @ref LL_RAMCFG_WRP_PAGE_110
  *         @arg @ref LL_RAMCFG_WRP_PAGE_111
  *         @arg @ref LL_RAMCFG_WRP_PAGE_112
  *         @arg @ref LL_RAMCFG_WRP_PAGE_113
  *         @arg @ref LL_RAMCFG_WRP_PAGE_114
  *         @arg @ref LL_RAMCFG_WRP_PAGE_115
  *         @arg @ref LL_RAMCFG_WRP_PAGE_116
  *         @arg @ref LL_RAMCFG_WRP_PAGE_117
  *         @arg @ref LL_RAMCFG_WRP_PAGE_118
  *         @arg @ref LL_RAMCFG_WRP_PAGE_119
  *         @arg @ref LL_RAMCFG_WRP_PAGE_120
  *         @arg @ref LL_RAMCFG_WRP_PAGE_121
  *         @arg @ref LL_RAMCFG_WRP_PAGE_122
  *         @arg @ref LL_RAMCFG_WRP_PAGE_123
  *         @arg @ref LL_RAMCFG_WRP_PAGE_124
  *         @arg @ref LL_RAMCFG_WRP_PAGE_125
  *         @arg @ref LL_RAMCFG_WRP_PAGE_126
  *         @arg @ref LL_RAMCFG_WRP_PAGE_127
  * @return The return value can be a 0x00000000U or combination of the following values :
  *         @arg @ref LL_RAMCFG_WRP_PAGE_96
  *         @arg @ref LL_RAMCFG_WRP_PAGE_97
  *         @arg @ref LL_RAMCFG_WRP_PAGE_98
  *         @arg @ref LL_RAMCFG_WRP_PAGE_99
  *         @arg @ref LL_RAMCFG_WRP_PAGE_100
  *         @arg @ref LL_RAMCFG_WRP_PAGE_101
  *         @arg @ref LL_RAMCFG_WRP_PAGE_102
  *         @arg @ref LL_RAMCFG_WRP_PAGE_103
  *         @arg @ref LL_RAMCFG_WRP_PAGE_104
  *         @arg @ref LL_RAMCFG_WRP_PAGE_105
  *         @arg @ref LL_RAMCFG_WRP_PAGE_106
  *         @arg @ref LL_RAMCFG_WRP_PAGE_107
  *         @arg @ref LL_RAMCFG_WRP_PAGE_108
  *         @arg @ref LL_RAMCFG_WRP_PAGE_109
  *         @arg @ref LL_RAMCFG_WRP_PAGE_110
  *         @arg @ref LL_RAMCFG_WRP_PAGE_111
  *         @arg @ref LL_RAMCFG_WRP_PAGE_112
  *         @arg @ref LL_RAMCFG_WRP_PAGE_113
  *         @arg @ref LL_RAMCFG_WRP_PAGE_114
  *         @arg @ref LL_RAMCFG_WRP_PAGE_115
  *         @arg @ref LL_RAMCFG_WRP_PAGE_116
  *         @arg @ref LL_RAMCFG_WRP_PAGE_117
  *         @arg @ref LL_RAMCFG_WRP_PAGE_118
  *         @arg @ref LL_RAMCFG_WRP_PAGE_119
  *         @arg @ref LL_RAMCFG_WRP_PAGE_120
  *         @arg @ref LL_RAMCFG_WRP_PAGE_121
  *         @arg @ref LL_RAMCFG_WRP_PAGE_122
  *         @arg @ref LL_RAMCFG_WRP_PAGE_123
  *         @arg @ref LL_RAMCFG_WRP_PAGE_124
  *         @arg @ref LL_RAMCFG_WRP_PAGE_125
  *         @arg @ref LL_RAMCFG_WRP_PAGE_126
  *         @arg @ref LL_RAMCFG_WRP_PAGE_127
  */
__STATIC_INLINE uint32_t LL_RAMCFG_GetPageWRP_96_127(const RAMCFG_TypeDef *p_ramcfg, uint32_t page_msk)
{
  return ((uint32_t)STM32_READ_BIT(p_ramcfg->WPR4, page_msk));
}
#endif /* LL_RAMCFG_WRP_PAGE_96 */

/**
  * @brief  Set the ramcfg unlock key for the ECC mechanism.
  * @rmtoll
  *  ECCKEYR           ECCKEY           LL_RAMCFG_SetECCKey
  * @param  p_ramcfg RAMCFG instance
  * @param  key
  *         This parameter must respect the following order:
  *         @arg @ref LL_RAMCFG_ECC_KEY_1
  *         @arg @ref LL_RAMCFG_ECC_KEY_2
  */
__STATIC_INLINE void LL_RAMCFG_SetECCKey(RAMCFG_TypeDef *p_ramcfg, uint32_t key)
{
  STM32_WRITE_REG(p_ramcfg->ECCKEYR, key);
}

/**
  * @brief  Set the ramcfg unlock key for the Erase operation.
  * @rmtoll
  *  ERKEYR           ERASEKEY           LL_RAMCFG_SetEraseKey
  * @param  p_ramcfg RAMCFG instance
  * @param  key
  *         This parameter must respect the following order:
  *         @arg @ref LL_RAMCFG_ERASE_KEY_1
  *         @arg @ref LL_RAMCFG_ERASE_KEY_2
  */
__STATIC_INLINE void LL_RAMCFG_SetEraseKey(RAMCFG_TypeDef *p_ramcfg, uint32_t key)
{
  STM32_WRITE_REG(p_ramcfg->ERKEYR, key);
}

/** @defgroup RAMCFG_LL_EF_FLAG_Management LL RAMCFG Flag Management
  * @{
  */

/**
  * @brief  Clear the RAMCFG pending flags.
  * @rmtoll
  *  ICR           CSEDC           LL_RAMCFG_ClearFlag \n
  *  ICR           CDED            LL_RAMCFG_ClearFlag
  * @param  p_ramcfg RAMCFG Instance.
  * @param  flags Flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref LL_RAMCFG_FLAG_SEDC    : ECC Single Error Detected and Corrected Flag.
  *            @arg @ref LL_RAMCFG_FLAG_DED     : ECC Double Error Detected Flag.
  *            @arg @ref LL_RAMCFG_FLAG_ECC_ALL : ECC Single Error Detected and Corrected Flag
                                                  and ECC Double Error Detected Flag.
  */
__STATIC_INLINE void LL_RAMCFG_ClearFlag(RAMCFG_TypeDef *p_ramcfg, uint32_t flags)
{
  STM32_WRITE_REG(p_ramcfg->ICR, flags);
}

/**
  * @brief  Clear the RAMCFG ECC Single Error Detected and Corrected flag.
  * @rmtoll
  *  ICR           CSEDC           LL_RAMCFG_ClearFlag_SEDC
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_ClearFlag_SEDC(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_WRITE_REG(p_ramcfg->ICR, LL_RAMCFG_FLAG_SEDC);
}

/**
  * @brief  Clear the RAMCFG ECC Double Error Detected flag.
  * @rmtoll
  *  ICR           CDED            LL_RAMCFG_ClearFlag_DED
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_ClearFlag_DED(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_WRITE_REG(p_ramcfg->ICR, LL_RAMCFG_FLAG_DED);
}

/**
  * @brief  Read the state of the RAMCFG flags.
  * @rmtoll
  *  ISR         SRAMBUSY         LL_RAMCFG_ReadFlag \n
  *  ISR           SEDC           LL_RAMCFG_ReadFlag \n
  *  ISR           DED            LL_RAMCFG_ReadFlag
  * @param  p_ramcfg RAMCFG Instance.
  * @param  flags Flags to read.
  *         This parameter can be one or a combination of the following values:
  *           @arg @ref LL_RAMCFG_FLAG_SEDC     : ECC Single Error Detected and Corrected Flag.
  *           @arg @ref LL_RAMCFG_FLAG_DED      : ECC Double Error Detected Flag.
  *           @arg @ref LL_RAMCFG_FLAG_ECC_ALL  : ECC Single Error Detected and Corrected Flag
                                                  and ECC Double Error Detected Flag.
              @arg @ref LL_RAMCFG_FLAG_SRAMBUSY : SRAM busy with erase operation Flag.
  * @retval State of selected flags.
            The return value can be one or a combination of the following values:
              @arg @ref LL_RAMCFG_FLAG_SEDC
              @arg @ref LL_RAMCFG_FLAG_DED
              @arg @ref LL_RAMCFG_FLAG_ECC_ALL
              @arg @ref LL_RAMCFG_FLAG_SRAMBUSY
  */
__STATIC_INLINE uint32_t LL_RAMCFG_ReadFlag(const RAMCFG_TypeDef *p_ramcfg, uint32_t flags)
{
  return STM32_READ_BIT(p_ramcfg->ISR, flags);
}

/**
  * @brief  Get the RAMCFG ECC Single Error Detected and Corrected flag.
  * @rmtoll
  *  ISR           SEDC               LL_RAMCFG_IsActiveFlag_SEDC
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of SE flag (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsActiveFlag_SEDC(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->ISR, LL_RAMCFG_FLAG_SEDC) == (LL_RAMCFG_FLAG_SEDC)) ? 1UL : 0UL);
}

/**
  * @brief  Get the RAMCFG ECC Double Error Detected flag.
  * @rmtoll
  *  ISR           DED                LL_RAMCFG_IsActiveFlag_DED
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of DE flag (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsActiveFlag_DED(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->ISR, LL_RAMCFG_FLAG_DED) == (LL_RAMCFG_FLAG_DED)) ? 1UL : 0UL);
}

/**
  * @brief  Get the RAMCFG SRAM Busy flag.
  * @rmtoll
  *  ISR           SRAMBUSY            LL_RAMCFG_IsActiveFlag_SRAMBUSY
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of SRAM Busy flag (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsActiveFlag_SRAMBUSY(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->ISR, LL_RAMCFG_FLAG_SRAMBUSY) == (LL_RAMCFG_FLAG_SRAMBUSY)) ? 1UL : 0UL);
}
/**
  * @}
  */

/** @defgroup RAMCFG_LL_EF_IT_Management LL RAMCFG IT Management
  * @{
  */

/**
  * @brief  Enable the specified RAMCFG interrupts.
  * @rmtoll
  *  IER           SEIE              LL_RAMCFG_EnableIT \n
  *  IER           DEIE              LL_RAMCFG_EnableIT \n
  *  IER           ECCNMI            LL_RAMCFG_EnableIT
  * @param  p_ramcfg RAMCFG Instance.
  * @param  mask Interrupt sources to enable.
  *         This parameter can be a combination of the following values:
  *           @arg @ref LL_RAMCFG_IT_SE    : Single Error Interrupt Mask.
  *           @arg @ref LL_RAMCFG_IT_DE    : Double Error Interrupt Mask.
  *           @arg @ref LL_RAMCFG_IT_NMI   : Double Error Interrupt redirection to NMI Mask.
  *           @arg @ref LL_RAMCFG_IT_ALL   : All Interrupt Mask.
  */
__STATIC_INLINE void LL_RAMCFG_EnableIT(RAMCFG_TypeDef *p_ramcfg, uint32_t mask)
{
  STM32_SET_BIT(p_ramcfg->IER, mask);
}

/**
  * @brief  Disable the specified RAMCFG interrupts.
  * @rmtoll
  *  IER           SEIE              LL_RAMCFG_DisableIT \n
  *  IER           DEIE              LL_RAMCFG_DisableIT
  * @param  p_ramcfg RAMCFG Instance.
  * @param  mask Interrupt sources to disable.
  *         This parameter can be a combination of the following values:
  *           @arg @ref LL_RAMCFG_IT_SE : Single Error Interrupt Mask.
  *           @arg @ref LL_RAMCFG_IT_DE : Double Error Interrupt Mask.
  * @note   LL_RAMCFG_IT_NMI is cleared only by a global RAMCFG reset.
  */
__STATIC_INLINE void LL_RAMCFG_DisableIT(RAMCFG_TypeDef *p_ramcfg, uint32_t mask)
{
  STM32_CLEAR_BIT(p_ramcfg->IER, mask);
}

/**
  * @brief  Enable the ECC single error RAMCFG interrupt.
  * @rmtoll
  *  IER           SEIE              LL_RAMCFG_EnableIT_SE
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_EnableIT_SE(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_SET_BIT(p_ramcfg->IER, LL_RAMCFG_IT_SE);
}

/**
  * @brief  Disable the ECC single error RAMCFG interrupt.
  * @rmtoll
  *  IER           SEIE              LL_RAMCFG_DisableIT_SE
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_DisableIT_SE(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_CLEAR_BIT(p_ramcfg->IER, LL_RAMCFG_IT_SE);
}

/**
  * @brief  Check whether the ECC single error RAMCFG interrupt is enabled.
  * @rmtoll
  *  IER           SEIE              LL_RAMCFG_IsEnabledIT_SE
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of interrupts sources (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledIT_SE(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->IER, LL_RAMCFG_IT_SE) == (LL_RAMCFG_IT_SE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the ECC double error RAMCFG interrupt.
  * @rmtoll
  *  IER           DEIE              LL_RAMCFG_EnableIT_DE
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_EnableIT_DE(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_SET_BIT(p_ramcfg->IER, LL_RAMCFG_IT_DE);
}

/**
  * @brief  Disable the ECC double error RAMCFG interrupt.
  * @rmtoll
  *  IER           DEIE              LL_RAMCFG_DisableIT_DE
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_DisableIT_DE(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_CLEAR_BIT(p_ramcfg->IER, LL_RAMCFG_IT_DE);
}

/**
  * @brief  Check whether the ECC double error RAMCFG interrupt is enabled.
  * @rmtoll
  *  IER           DEIE              LL_RAMCFG_IsEnabledIT_DE
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of interrupts sources (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledIT_DE(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->IER, LL_RAMCFG_IT_DE) == (LL_RAMCFG_IT_DE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the ECC double error redirected to NMI interrupt.
  * @rmtoll
  *  IER           ECCNMI            LL_RAMCFG_EnableIT_NMI
  * @param  p_ramcfg RAMCFG Instance.
  */
__STATIC_INLINE void LL_RAMCFG_EnableIT_NMI(RAMCFG_TypeDef *p_ramcfg)
{
  STM32_SET_BIT(p_ramcfg->IER, LL_RAMCFG_IT_NMI);
}

/**
  * @brief  Check whether the ECC double error redirected to NMI interrupt is enabled.
  * @rmtoll
  *  IER           ECCNMI            LL_RAMCFG_IsEnabledIT_NMI
  * @param  p_ramcfg RAMCFG Instance.
  * @return State of interrupts sources (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RAMCFG_IsEnabledIT_NMI(const RAMCFG_TypeDef *p_ramcfg)
{
  return ((STM32_READ_BIT(p_ramcfg->IER, LL_RAMCFG_IT_NMI) == (LL_RAMCFG_IT_NMI)) ? 1UL : 0UL);
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_RAMCFG_H */
