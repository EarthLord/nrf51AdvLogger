/**
 * @defgroup board-config Board Configuration
 * Configuration for different boards
 * @{
 *
 * @file
 * This file contains all the board specific configurations, PCA10000 in this case.
 * PCA1000's RGB LED and UART pin configurations are present.
 *
 * @author
 * Prithvi
 */

#ifndef BOARD_H
#define BOARD_H
 
#ifdef BOARD_PCA10000
/** @anchor led-pin-definitions
 * @name Definitions for the RGB LED's pins in PCA10000
 * Used in @ref led-driver module
 * @{*/
/** Red LED pin number */
#define LED_RGB_RED    21
/** Green LED pin number */
#define LED_RGB_GREEN  22
/** Blue LED pin number */
#define LED_RGB_BLUE   23
/** @} */

/** @anchor uart-pin-definitions
 * @name Definitions for the UART pins in PCA10000
 * Used in @ref uart-driver module
 * @{*/
/** Rx pin number */
#define RX_PIN_NUMBER  11
/** Tx pin number */
#define TX_PIN_NUMBER  9
/** @brief CTS pin number.
 * Used if @ref HWFC is true */
#define CTS_PIN_NUMBER 10
/** @brief RTS pin number.
 * Used if @ref HWFC is true */
#define RTS_PIN_NUMBER 8
/** @brief Specify with a boolean value if Hardware Flow Control is required.
 *  PCA1000 cannot work without HWFC */
#define HWFC           true
/** @}
 */

#elif ANOTHER_BOARD
/*	Define board specific pins and configurations here */
#endif  /* BOARD_PCA10000 */

#endif
/** @}
 */
