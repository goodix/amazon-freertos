/**
 ****************************************************************************************
 *
 * @file ble_gapc.h
 *
 * @brief BLE GAPC API
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix Technology Co., Ltd
 * All Rights Reserved
 *
 ****************************************************************************************
 */

   /**
 * @addtogroup BLE
 * @{
 * @brief Definitions and prototypes for the BLE SDK interface.
 */
 
  /**
 * @addtogroup BLE_GAP Generic Access Profile (GAP)
 * @{
 * @brief Definitions and prototypes for the GAP interface.
 */
 
 /**
 * @defgroup BLE_GAPC Generic Access Profile (GAP) Connection Control
 * @{
 * @brief Definitions and prototypes for the GAP Connection Control interface.
 */
#ifndef __BLE_GAPC_H__
#define __BLE_GAPC_H__

#include "ble_error.h"
#include "gr55xx_sys_cfg.h"
#include <stdint.h>         // Standard Integer
#include <string.h>
#include <stdbool.h>


/**
 * @defgroup  BLE_GAPC_DEFINES Defines
 * @{
 */
#define GAP_CHNL_MAP_LEN         0x05 /**< The length of channel map. */
#define GAP_FEATS_LEN            0x08 /**< The length of features. */
#define GAP_ADDR_LEN             0x06 /**< The length of address. */
#define GAP_INVALID_CONN_INDEX   0xFF /**< Invalid connection index. */

#if defined(GR551xx_D0)
#define GAP_MIN_IQ_SAMPLE_NUM    0x09
#define GAP_MAX_IQ_SAMPLE_NUM    0x52
#endif


/**@defgroup BLE_GAP_ADDR_TYPES GAP Address types
 * @{ */
#define BLE_GAP_ADDR_TYPE_PUBLIC                        0x00 /**< Public (identity) address.*/
#define BLE_GAP_ADDR_TYPE_RANDOM_STATIC                 0x01 /**< Random static (identity) address. */
/**@} */


/**@defgroup BLE_GAP_PHY_OPTIONS GAP PHY OPTIONS (bitmask)
 * @{ */
#define PHY_OPT_NO_CODING    0x00       /**< The Host has no preferred coding when transmitting on the LE Coded PHY. */
#define PHY_OPT_S2_CODING    0x01       /**< The Host prefers that S=2 coding be used when transmitting on the LE Coded PHY. */
#define PHY_OPT_S8_CODING    0x02       /**< The Host prefers that S=8 coding be used when transmitting on the LE Coded PHY. */
/**@} */

/** @} */

/**
 * @defgroup BLE_SDK_GAP_ENUM Enumerations
 * @{
 */
/** @brief The operation code used to get connection info */
typedef enum  
{
    GAP_GET_CON_RSSI = 0,        /**< Get connection RSSI info. */
    GAP_GET_CON_CHANNEL_MAP,     /**< Get connection channel map. */
    GAP_GET_PHY,                 /**< Get connection PHY. */
    GAP_GET_CHAN_SEL_ALGO        /**< Get selection algorithm for connection channel. */
} gap_get_conn_info_op_t;

/**@brief The operation code used to get peer device info. */
typedef enum 
{
    GAP_GET_PEER_VERSION = 0,    /**< Get peer device version info. */
    GAP_GET_PEER_FEATURES        /**< Get peer device features info. */
} gap_get_peer_info_op_t;

/** @brief Advertising report type. */
typedef enum
{
    GAP_REPORT_TYPE_ADV_EXT = 0,  /**< Extended advertising report. */
    GAP_REPORT_TYPE_ADV_LEG,      /**< Legacy advertising report. */
    GAP_REPORT_TYPE_SCAN_RSP_EXT, /**< Extended scan response report. */
    GAP_REPORT_TYPE_SCAN_RSP_LEG, /**< Legacy scan response report. */
    GAP_REPORT_TYPE_PER_ADV,      /**< Periodic advertising report. */
} gap_adv_report_type_t;

/** @brief Advertising report information. */
typedef enum
{
    GAP_REPORT_INFO_COMPLETE_BIT = (1 << 0), /**< Report is complete. */
    GAP_REPORT_INFO_CONN_ADV_BIT = (1 << 1), /**< Connectable advertising. */
    GAP_REPORT_INFO_SCAN_ADV_BIT = (1 << 2), /**< Scannable advertising. */
    GAP_REPORT_INFO_DIR_ADV_BIT  = (1 << 3), /**< Directed advertising. */
} gap_adv_report_info_t;

/** @brief Stop reason code. */
typedef enum
{
    GAP_STOPPED_REASON_TIMEOUT = 0,          /**< Stop with timeout. */
    GAP_STOPPED_REASON_ON_USER,              /**< Stop with user stopping it actively. */
    GAP_STOPPED_REASON_CONN_EST              /**< Stop with connection established. */
} gap_stopped_reason_t;

/** @brief Device role of LL layer type */
typedef enum
{
    GAP_LL_ROLE_MASTER = 0,                  /**< Master role. */
    GAP_LL_ROLE_SLAVE  = 1,                  /**< Slave role. */
} gap_ll_role_type_t;

/**
 * @brief Operation code used to set param(s).
 */
typedef enum
{
    GAP_OPCODE_CHNL_MAP_SET,            /**< Set Channel Map. */
    GAP_OPCODE_WHITELIST_SET,           /**< Set white list. */
    GAP_OPCODE_PER_ADV_LIST_SET,        /**< Set periodic advertising list. */
    GAP_OPCODE_PRIVACY_MODE_SET,        /**< Set privacy mode for peer device. */
} gap_param_set_op_id_t;

/**
 * @brief Operation code used to read resolvable address.
 */
typedef enum
{
    GAP_OPCODE_LOCAL_RSLV_ADDR_READ,    /**< Local resolvable address operation. */
    GAP_OPCODE_PEER_RSLV_ADDR_READ,     /**< Peer resolvable address operation. */
} gap_rslv_addr_read_op_id_t;

/**
 * @brief Operation code used for LEPSM manager.
 */
typedef enum
{
    GAP_OPCODE_LEPSM_REGISTER,      /**< LEPSM register operation. */
    GAP_OPCODE_LEPSM_UNREGISTER,    /**< LEPSM unregister operation. */
} gap_psm_manager_op_id_t;


#if defined(GR551xx_D0)

/**
 * @brief Type of constant tone extension.
 */
typedef enum
{
    GAP_CET_AOA,      /**< Allow AoA Constant Tone Extension Response. */
    GAP_CET_AOD_1US,  /**< Allow AoD Constant Tone Extension Response with 1 ¦Ìs slots. */
    GAP_CET_AOD_2US   /**< Allow AoD Constant Tone Extension Response with 2 ¦Ìs slots. */
} gap_cte_type_t;

/**
 * @brief Type of switching and sampling slots 
 */
typedef enum
{
    GAP_SLOT_1US = 0x01,     /**< Switching and sampling slots are 1 ¦Ìs each. */
    GAP_SLOT_2US,            /**< Switching and sampling slots are 2 ¦Ìs each. */
} gap_switching_sampling_type_t;

/**
 * @brief Status of IQ report packet
 */
typedef enum
{
    GAP_CRC_OK,                      /**< CRC was correct. */
    GAP_CRC_ERR1,                    /**< CRC was incorrect and the Length and CTETime fields of the packet were used to determine sampling points. */
    GAP_CRC_ERR2,                    /**< CRC was incorrect but the Controller has determined the position and length of the Constant Tone Extension in some other way. */
    GAP_INSUFFI_RESOURCE = 0xFF     /**< Insufficient resources to sample (data_channel_idx, cte_type, and slot_dur invalid). */
} gap_iq_report_status_t;

/**
 * @brief Phy for power control management 
 */
 typedef enum
{
    GAP_PHY_1M       = 0x01,        /**< LE 1M PHY. */
    GAP_PHY_2M       = 0x02,        /**< LE 2M PHY. */
    GAP_PHY_CODED_S8 = 0x03,        /**< LE Coded PHY with S=8 data coding. */
    GAP_PHY_CODED_S2 = 0x04         /**< LE Coded PHY with S=2 data coding. */
} gap_phy_type_t;

/**
 * @brief Transmit power change reporting reason.
 */
typedef enum
{
    GAP_PWR_LOCAL_TX_CHG   = 0x00, /**< Local transmit power changed. */
    GAP_PWR_REMOTE_TX_CHG  = 0x01, /**< Remote transmit power changed. */
} gap_tx_pwr_change_report_reason_t;

/**
 * @brief Transmit Power level flag.
 */
typedef enum
{
    GAP_PWR_MID_LVL  = 0x00, /**< Transmit power level is between minimum and max level. */
    GAP_PWR_MIN_LVL  = 0x01, /**< Transmit power level is at minimum level. */
    GAP_PWR_MAX_LVL  = 0x02  /**< Transmit power level is at maximum level. */
} gap_pwr_lvl_flag_t;

/// Path Loss zones. HCI:7.8.118
typedef enum
{
    GAP_PATH_LOSS_LOW           = 0x00, /**< Entered Low zone. */
    GAP_PATH_LOSS_MID           = 0x01, /**< Entered Middle zone. */
    GAP_PATH_LOSS_HIGH          = 0x02, /**< Entered High zone. */
} gap_path_loss_zone_t;

#endif

/** @} */


/**
 * @defgroup BLE_GAPC_STRUCT Structures
 * @{
 */

/** @brief The struct of device version. */
typedef struct
{
    uint8_t  hci_ver;      /**< HCI version. */
    uint8_t  lmp_ver;      /**< LMP version. */
    uint8_t  host_ver;     /**< Host version. */
    uint16_t hci_subver;   /**< HCI subversion. */
    uint16_t lmp_subver;   /**< LMP subversion. */
    uint16_t host_subver;  /**< Host subversion. */
    uint16_t manuf_name;   /**< Manufacturer name. */
} gap_dev_version_ind_t;

/** @brief The struct of address. */
typedef struct
{
    uint8_t  addr[GAP_ADDR_LEN]; /**< 6-byte array address value. */
} gap_addr_t;

/** @brief The struct of broadcast address with broadcast type. */
typedef struct
{
    gap_addr_t gap_addr;     /**< Device BD Address. */
    uint8_t    addr_type;    /**< Address type of the device: 0=public/1=random. please @ref BLE_GAP_ADDR_TYPES. */
} gap_bdaddr_t;

/** @brief Get broadcast address struct. */
typedef struct
{
    uint8_t     index;        /**< Advertsing index. The valid range is: 0 - 4. */
    gap_bdaddr_t bd_addr;     /**< BD address. */
} gap_get_bd_addr_t;

/** @brief TX power info struct. */
typedef struct
{
    int8_t     power_lvl;       /**< Advertising channel TX power level. Range: -20 to 10. Unit: dBm. Accuracy: +/-4dB. */
} gap_dev_adv_tx_power_t;

/** @brief TX power info struct. */
typedef struct
{
    int8_t min_tx_pwr;      /**< MIN of TX power. Size: 1 octet (signed integer). Range: -127  to +126. Unit: dBm. */
    int8_t max_tx_pwr;      /**< MAX of TX power. Size: 1 octet (signed integer). Range: -127 to +126. Unit: dBm. */
} gap_dev_tx_power_t;

/** @brief Max data length info struct. */
typedef struct
{
    uint16_t suppted_max_tx_octets; /**< Maximum number of payload octets that the local Controller supports for transmission of a single Link Layer packet on a data connection.
                                         Range: 0x001B-0x00FB (all other values reserved for future use). */
    uint16_t suppted_max_tx_time;   /**< Maximum time, in microseconds, that the local Controller supports for transmission of a single Link Layer packet on a data connection.
                                         Range: 0x0148-0x4290 (all other values reserved for future use). */
    uint16_t suppted_max_rx_octets; /**< Maximum number of payload octets that the local Controller supports for reception of a single Link Layer packet on a data connection.
                                         Range: 0x001B-0x00FB (all other values reserved for future use). */
    uint16_t suppted_max_rx_time;   /**< Maximum time, in microseconds, that the local Controller supports for reception of a single Link Layer packet on a data connection.
                                         Range: 0x0148-0x4290 (all other values reserved for future use). */
} gap_max_data_len_t;

/** @brief Suggested default data length info. */
typedef struct
{
    uint16_t suggted_max_tx_octets; /**< The Host's suggested value for the Controller's maximum transmitted number of payload octets to be used for new connections.
                                         Range: 0x001B-0x00FB (all other values reserved for future use), default: 0x001B */
    uint16_t suggted_max_tx_time;   /**< The Host's suggested value for the Controller's maximum packet transmission time to be used for new connections.
                                         Range: 0x0148-0x4290 (all other values reserved for future use), default: 0x0148*/
} gap_sugg_dflt_data_len_t;

/** @brief Number of available advertising sets info. */
typedef struct
{
    uint8_t nb_adv_sets; /**< Number of available advertising sets. */
} gap_nb_adv_sets_t;

/** @brief Maximum advertising data length info. */
typedef struct
{
    uint16_t length; /**< Maximum advertising data length supported by controller. */
} gap_max_adv_data_len_ind_t;

/** @brief RF path compensation values info. */
typedef struct
{
    uint16_t tx_path_comp; /**< RF TX path compensation. */
    uint16_t rx_path_comp; /**< RF RX path compensation. */
} gap_dev_rf_path_comp_ind_t;

#if defined(GR551xx_D0)
/** @brief antenna information. */
typedef struct
{
    uint8_t     supp_switching_sampl_rates;  /**< Supported switching sampling rates bit field (@see enum gap_switch_sampling_rate). */
    uint8_t     antennae_num;                /**< Number of antennae, range 0x01 to 0x4B. */
    uint8_t     max_switching_pattern_len;   /**< Max length of switching pattern (number of antenna IDs in the pattern), range 0x02 to 0x4B. */
    uint8_t     max_cte_len;                 /**< Max CTE length, range 0x02 to 0x14. */
} gap_antenna_inf_t;
#endif

/** @brief Device info. */
typedef union
{
    gap_dev_version_ind_t       dev_version;            /**< Version info. */
    gap_get_bd_addr_t           get_bd_addr;            /**< Device BD address info. */   
    gap_dev_adv_tx_power_t      adv_tx_power;           /**< Advertising TX power info. */
    gap_sugg_dflt_data_len_t    sugg_dflt_data_len;     /**< Suggested default data length info. */
    gap_max_data_len_t          max_data_len;           /**< Suggested  MAX data length info. */
    gap_nb_adv_sets_t           nb_adv_sets;            /**< Number of available advertising sets. */
    gap_max_adv_data_len_ind_t  max_adv_data_len;       /**< Maximum advertising data length info. */
    gap_dev_tx_power_t          dev_tx_power;           /**< Device TX power info. */
    gap_dev_rf_path_comp_ind_t  dev_rf_path_comp;       /**< RF path compensation values. */

    #if defined(GR551xx_D0)
    gap_antenna_inf_t           dev_antenna_inf;        /**< Device antenna information. */
    #endif
} gap_dev_info_t;

/** @brief Get device info operation struct. */
typedef struct
{
    uint8_t           operation;      /**< Operation code. @see enum gap_dev_info_get_type_t. */
    gap_dev_info_t    dev_info;       /**< Device info. */
 } gap_dev_info_get_t;

/** @brief Read resolvable address info struct. */
typedef struct
{
    uint8_t           op_code;        /**< Operation code. @see enum gap_rslv_addr_read_op_id_t. */
    gap_addr_t        gap_addr;       /**< Resolvable address info. */
 } gap_rslv_addr_read_t;

/** @brief Sync established indication. */
typedef struct
{
    uint8_t      phy;           /**< PHY on which synchronization has been established. @see gap_phy_type. */
    uint16_t     intv;          /**< Periodic advertising interval (in unit of 1.25ms, min is 7.5ms). */
    uint8_t      adv_sid;       /**< Advertising SID. */
    uint8_t      clk_acc;       /**< Advertiser clock accuracy. @see enum gapm_clk_acc. */
    gap_bdaddr_t bd_addr;       /**< Advertiser address. */
    uint16_t sync_hdl;          /**< Sync handle. */

#if defined(GR551xx_D0)
    uint16_t serv_data;         /**< Service data. */
#endif

} gap_sync_established_ind_t;


/** @brief APP receives the extended advertising report indication info struct. */
typedef struct
{
    uint8_t      adv_type;              /**< Advertising type. @see enum gap_adv_report_type_t. */
    uint8_t      adv_info;              /**< Bit field providing information about the received report. @see enum gap_adv_report_info_t. */
    gap_bdaddr_t broadcaster_addr;      /**< Broadcaster device address. */
    gap_bdaddr_t direct_addr;           /**< Target address (in case of a directed advertising report). */
    int8_t       tx_pwr;                /**< TX power (in dBm). */
    int8_t       rssi;                  /**< RSSI (between -127 and +20 dBm). */
    uint8_t      phy_prim;              /**< Primary PHY on which advertising report has been received. */
    uint8_t      phy_second;            /**< Secondary PHY on which advertising report has been received. */
    uint8_t      adv_sid;               /**< Advertising SID , valid only for periodic advertising report. */
    uint16_t     period_adv_intv;       /**< Periodic advertising interval (in unit of 1.25ms, min is 7.5ms), valid only for periodic advertising report. */
    uint8_t      per_sync_idx;          /**< Periodic syncronization index, valid only for periodic advertising report. */
    uint16_t     length;                /**< Report length. */
    uint8_t      data[__ARRAY_EMPTY];   /**< Report. */
} gap_ext_adv_report_ind_t;

/** @brief  Name of peer device indication. */
typedef struct
{
    gap_addr_t  peer_addr;              /**< Peer device bd address. */
    uint8_t     addr_type;              /**< Peer device address type. */
    uint8_t     name_len;               /**< Peer device name length. */
    uint8_t     name[__ARRAY_EMPTY];    /**< Peer device name. */
} gap_peer_name_ind_t;

/** @brief Connection parameter used to update connection parameters. */
typedef struct
{
    uint16_t interval;           /**< Connection interval. Range: 0x0006 to 0x0C80. Unit: 1.25 ms. Time range: 7.5 ms to 4 s. */
    uint16_t latency;            /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t sup_timeout;        /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} gap_conn_update_cmp_t;

/** @brief The parameter of connection. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} gap_conn_param_t;


/** @brief The parameter of update connection. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. range: 0x000A to 0x0C80, unit: 10 ms, Time range: 100 ms to 32 s. */
     uint16_t ce_len;        /**< The length of connection event needed for this LE connection. Range: 0x0002 to 0xFFFF, unit: 0.625 ms, time Range: 1.25 ms to 40.9 s. */
} gap_conn_update_param_t;

/** @brief  Connection complete info. */
typedef struct
{
    uint16_t             conhdl;            /**< Connection_Handle. Range: 0x0000-0x0EFF (all other values reserved for future use). */
    uint16_t            con_interval;       /**< Connection interval. Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s. */
    uint16_t            con_latency;        /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t            sup_to;             /**< Connection supervision timeout. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
    uint8_t             clk_accuracy;       /**< Clock accuracy (0x00: 500 ppm, 0x01: 250 ppm, 0x02: 150 ppm, 0x03: 100 ppm, 0x04: 75 ppm, 
                                                 0x05:50 ppm, 0x06:30 ppm, 0x07:20 ppm, others: reserved for future use). */
    uint8_t             peer_addr_type;     /**< Peer address type(0x00: Public Device Address, 0x01 : Random Device Address, others: reserved for future use). */
    gap_addr_t          peer_addr;          /**< Peer BT address. */
    gap_ll_role_type_t  ll_role;            /**< Device Role of LL Layer. */
} gap_conn_cmp_t;

/** @brief  Channel map structure. */
typedef struct
{
    uint8_t map[GAP_CHNL_MAP_LEN]; /**< This parameter contains 37 1-bit fields. The nth bit (n is in the range of 0 to 36) contains the value for the link layer channel index n.
                                        Channel n is unused = 0, channel n is used = 1. The most significant bits are reserved for future use.*/
} gap_chnl_map_t;

/** @brief PHY info. */
typedef struct
{
    uint8_t tx_phy; /**< LE PHY for data transmission. @see BLE_GAP_PHYS. */
    uint8_t rx_phy; /**< LE PHY for data reception. @see BLE_GAP_PHYS. */
} gap_le_phy_ind_t;

/** @brief Connection info. */
typedef union
{
    int8_t           rssi;              /**< RSSI. */
    gap_chnl_map_t   chnl_map;          /**< channel map. */
    gap_le_phy_ind_t phy;               /**< PHY indicaiton. */
    uint8_t          chan_sel_algo;     /**< Chanel Selection algorithm, 0x00: LE Channel Selection Algorithm #1 is used.
                                             0x01: LE Channel Selection Algorithm #2 is used.\n 0x02-0xFF: reserved. */
} gap_conn_info_t;

/** @brief The info of connecting operation. */
typedef struct
{
    uint8_t         opcode;     /**< Operation code. See @ref gap_get_conn_info_op_t. */
    gap_conn_info_t info;       /**< Connection info. */
} gap_conn_info_param_t;

/** @brief Peer version info. */
typedef struct 
{
    uint16_t compid;        /**<Manufacturer name. */
    uint16_t lmp_subvers;   /**< LMP subversion. */
    uint8_t  lmp_vers;      /**< LMP version. */
} gap_peer_version_ind_t;


/** @brief LE features info. */
typedef struct
{
    uint8_t features[GAP_FEATS_LEN]; /**< 8-byte array for LE features\n 
                                          Feature Setting field's bit mapping to Controller Features (0: not support, 1: support) \n
                                                          |Bit position       | Link Layer Feature|
                                                          |-------------|-----------------|
                                                          |0                    | LE Encryption|
                                                          |1                    |Connection Parameters Request Procedure| 
                                                          |2                    |Extended Reject Indication|
                                                          |3                    | Slave-initiated Features Exchange | 
                                                          |4                    |LE Ping | 
                                                          |5                    |LE Data Packet Length Extension | 
                                                          |6                    |LL Privacy |  
                                                          |7                    |Extended Scanner Filter Policies | 
                                                          |8                    |LE 2M PHY|  
                                                          |9                    | Stable Modulation Index - Transmitter | 
                                                          |10                   | Stable Modulation Index - Receiver |
                                                          |11                   |LE Coded PHY | 
                                                          |12                   |LE Extended Advertising| 
                                                          |13                   | LE Periodic Advertising| 
                                                          |14                   | Channel Selection Algorithm #2| 
                                                          |15                    |LE Power Class 1|
                                                          |16                  |Minimum Number of Used Channels Procedure|  
                                                          |All other values |Reserved for Future Use|*/
}gap_peer_features_ind_t;

/** @brief LE peer info. */
typedef union
{
    gap_peer_version_ind_t  peer_version;   /**< Version info. */
    gap_peer_features_ind_t peer_features;  /**< Features info. */
} gap_peer_info_t;

/** @brief Get peer info operation struct. */
typedef struct
{
    uint8_t         opcode;         /**< Operation code. See @ref gap_get_peer_info_op_t. */
    gap_peer_info_t peer_info;      /**< Peer info. */
} gap_peer_info_param_t;

/** @brief Supported data length size Indication. */
typedef struct
{
    uint16_t max_tx_octets; /**<  The maximum number of payload octets in TX. */
    uint16_t max_tx_time;   /**<  The maximum time that the local Controller will take to TX. */
    uint16_t max_rx_octets; /**<  The maximum number of payload octets in RX. */
    uint16_t max_rx_time;   /**<  The maximum time that the local Controller will take to RX. */
} gap_le_pkt_size_ind_t;

/**@brief The Structure for BLE Connection Arrangement. */
typedef struct
{
    uint16_t conn_idx;     /**< Connection Index. */
    uint32_t interval;     /**< Connection Interval (in 625 ¦Ìs). */
    uint32_t offset;       /**< Connection Offset (in 625 ¦Ìs). */
    uint32_t duration;     /**< Connection Duration (in 625 ¦Ìs). */
} gap_con_plan_tag_t;

/** @brief Set preference slave event duration */
typedef struct
{
    uint16_t duration; /**< Preferred event duration. */
    uint8_t  single_tx; /**< Slave transmits a single packet per connection event (False/True). */
} gap_set_pref_slave_evt_dur_param_t;

#if defined(GR551xx_D0)

/**
 * @brief Connectionless IQ Report info
 */
typedef struct
{
    uint8_t  channel_idx;                     /**< The index of the channel on which the packet was received, range 0x00 to 0x24. */
    int16_t  rssi;                            /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                 /**< RSSI antenna ID. */
    uint8_t  cte_type;                        /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum gap_cte_type_t. */
    uint8_t  slot_dur;                        /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                      /**< Packet status, @see enum gap_iq_report_status_t. */
    uint16_t pa_evt_cnt;                      /**< Periodic advertising event counter. */
    uint8_t  nb_samples;                      /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                   0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} gap_connless_iq_report_t;

/** @brief Set connection CTE transmit parameters info. */
typedef struct
{
    uint8_t cte_type;     /**< The type of cte, see @ref gap_cte_type_t. */
    uint8_t num_antenna;  /**< The number of Antenna IDs in the pattern, range 0x02 to 0x4B. */
    uint8_t *antenna_id;  /**< List of Antenna IDs in the pattern. */
} gap_set_conn_cte_trans_param_t;

/** @brief Set connection CTE receive parameters info. */
typedef struct
{
    bool    sampling_enable; /**< Wheter to sample IQ from the CTE. */
    uint8_t slot_durations;  /**< The slot for sample IQ from the CTE, see @ref gap_switching_sampling_type_t. */
    uint8_t num_antenna;     /**< The number of Antenna IDs in the pattern, range 0x02 to 0x4B. */
    uint8_t *antenna_id;     /**< List of Antenna IDs in the pattern. */
} gap_set_conn_cte_rcv_param_t;

/** @brief Set connection CTE Request enable info. */
typedef struct
{
    uint16_t cte_req_interval;    /**< Defines whether the cte request procedure is initiated only once or periodically.
                                       0x0000: initiate the Constant Tone Extension Request procedure once.
                                       0x0001 to 0xFFFF: requested interval for initiating the cte request procedure in number of connection events. */
    uint8_t  cte_req_len;         /**< Minimum length of the cte being requested in 8us units, range 0x02 to 0x14. */
    uint8_t  cte_req_type;        /**< The type for requested cte, see @ref gap_cte_type_t. */
} gap_set_conn_cte_req_enable_t;

/** @brief Connection IQ Report info. */
typedef struct
{
    uint8_t  rx_phy;                          /**< Rx PHY (0x01: 1M | 0x02: 2M), see @ref BLE_GAP_PHYS. */
    uint8_t  data_channel_idx;                /**< Data channel index, range 0x00 to 0x24. */
    int16_t  rssi;                            /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                 /**< RSSI antenna ID. */
    uint8_t  cte_type;                        /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum gap_cte_type_t. */
    uint8_t  slot_dur;                        /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                      /**< Packet status, @see enum gap_iq_report_status_t. */
    uint16_t con_evt_cnt;                     /**< Connection event counter. */
    uint8_t  nb_samples;                      /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                   0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} gap_conn_iq_report_t;

/** @brief Set path loss reporting parameter info. */
typedef struct
{
    uint8_t  high_thr;                  /**< High threshold for the path loss (dB). */
    uint8_t  high_hyst;                 /**< Hysteresis value for the high threshold (dB). */
    uint8_t  low_thr;                   /**< Low threshold for the path loss (dB). */
    uint8_t  low_hyst;                  /**< Hysteresis value for the low threshold (dB). */
    uint16_t min_conn_evt_num;          /**< Minimum time in number of connection events to be observed. */
} gap_set_path_loss_report_param_t;

/** @brief Transmit power change reporting info. */
typedef struct
{
    uint8_t  reason;    /**< Reason see @ref gap_tx_pwr_change_report_reason_t. */
    uint8_t  phy;       /**< Phy see @ref gap_phy_type_t. */
    int8_t   tx_pwr;    /**< Transmit Power level (dBm). */
    uint8_t  flags;     /**< Transmit Power level flags, see @ref gap_pwr_lvl_flag_t. */
    int8_t   delta;     /**< Change in transmit power level (dBm). */
} gap_tx_pwr_change_report_t;

/** @brief Path loss threshold reporting info. */
typedef struct
{
    uint8_t curr_path_loss;   /**< Current path loss (dB). */
    uint8_t zone_entered;     /**< Zone entered, see @ref gap_path_loss_zone_t. */
} gap_path_loss_threshold_report_t;

/** @brief Local transmit power read indication info. */
typedef struct
{
    uint8_t phy;              /**< Phy see @ref gap_phy_type_t. */
    int8_t  curr_tx_pwr_lvl;  /**< Current transmit power level (dBm). */
    int8_t  max_tx_pwr_lvl;   /**< Max transmit power level (dBm). */
} gap_local_tx_pwr_read_ind_t;

/** @brief Remote transmit power read indication info. */
typedef struct
{
    uint8_t  phy;       /**< Phy see @ref gap_phy_type_t. */
    int8_t   tx_pwr;    /**< Transmit Power level (dBm). */
    uint8_t  flags;     /**< Transmit Power level flags, see @ref gap_pwr_lvl_flag_t. */
} gap_remote_tx_pwr_read_ind_t;

#endif

/** @brief BLE initialization completed callback function for application. */
typedef void (*app_ble_init_cmp_cb_t)(void);

/** @brief The gap callback function struct. */
typedef struct
{  
    // ----------------------------------------------------------------------------------
    // -------------------------  Common Callbacks       ------------------------------------
    // ----------------------------------------------------------------------------------

    /**
      ****************************************************************************************
      * @brief This callback function will be called when the set param(s) operation has been completed.
      * @param[in] status:  The status of set param operation.
      * @param[in] set_param_op: The operation of setting. @see gap_param_set_op_id_t.
      * @retval void
      ****************************************************************************************
      */
    void (*app_gap_param_set_cb)(uint8_t status, const gap_param_set_op_id_t set_param_op);
    
    /**
      ****************************************************************************************
      * @brief This callback function will be called when the PSM register/unregister operation has been completed.
      * @param[in] status:  The status of PSM manager operations.
      * @param[in] set_param_op: The operation of register/unregister PSM. @see gap_psm_op_id_t
      * @retval void
      ****************************************************************************************
      */
    void (*app_gap_psm_manager_cb)(uint8_t status, const gap_psm_manager_op_id_t psm_op);

     /**
      ****************************************************************************************
      * @brief This callback function will be called when update PHY completed.
      * @param[in] status:  The index of connections.
      * @param[in] status:  The status of udpate PHY operation.
      * @param[in] phy:     The PHY info.
      * @retval void
      ****************************************************************************************
      */

    void (*app_gap_phy_update_cb)(uint8_t conn_idx, uint8_t status, const gap_le_phy_ind_t *p_phy);

    /**
      ****************************************************************************************
      * @brief This callback function will be called once the requested parameters has been available.
      * @param[in] status: GAP operation status.
      * @param[in] p_dev_info: The device info. See @ref gap_dev_info_get_t.
      * @retval void
      ****************************************************************************************
      */
    void (*app_gap_dev_info_get_cb)(uint8_t status, const gap_dev_info_get_t* p_dev_info);
    

    // ----------------------------------------------------------------------------------
    // -------------------------  Advertising Callbacks       ----------------------------------
    // ----------------------------------------------------------------------------------
    
    /**
     ****************************************************************************************
     * @brief This callback function will be called when the adv has started.
     * @param[in] inst_idx:        The advertising index. Valid range is: 0 - 4.
     * @param[in] status:          The status of starting a advertiser.
     * @retval void
     ****************************************************************************************
     */      
    void (*app_gap_adv_start_cb)(uint8_t inst_idx, uint8_t status);
    
    /**
     ****************************************************************************************
     * @brief This callback function will be called when the adv has stopped.
     * @param[in] inst_idx:        The advertising index. Valid range is: 0 - 4.
     * @param[in] status:          The status of stopping a advertiser. If status is not success, adv_stop_reason is invalid.
     * @param[in] adv_stop_reason: The stop reason. See @ref gap_stopped_reason_t.
     * @retval void
     ****************************************************************************************
     */      
    void (*app_gap_adv_stop_cb)(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t adv_stop_reason);

    /**
     ****************************************************************************************
     * @brief This callback function will be called when app has received the scan request.
     * @param[in] inst_idx:       The advertising index. Valid range is: 0 - 4.
     * @param[in] p_scanner_addr: The BD address. See @ref gap_bdaddr_t.
     * @retval void
     ****************************************************************************************
     */  
    void (*app_gap_scan_req_ind_cb)(uint8_t inst_idx, const gap_bdaddr_t *p_scanner_addr); 

    /**
     ****************************************************************************************
     * @brief This callback function will be called when update adv data completed.
     * @param[in] inst_idx:The advertising index. Valid range is: 0 - 4.
     * @param[in] status:  The status of udpate PHY operation.
     * @retval void
     ****************************************************************************************
     */
    void (*app_gap_adv_data_update_cb)(uint8_t inst_idx, uint8_t status);

    // ----------------------------------------------------------------------------------
    // --------------------  Scanning/Periodic Synchronization Callbacks  ------------------------
    // ----------------------------------------------------------------------------------

    /**
     ****************************************************************************************
     * @brief This callback function will be called when the scan has started.
     * @param[in] status:          The status of starting a scanner.
     * @retval void
     ****************************************************************************************
     */      
    void (*app_gap_scan_start_cb)(uint8_t status);

   /**
     ****************************************************************************************
     * @brief This callback function will be called once the scanning activity has been stopped.
     * @param[in] status:           The status of stopping a scanner.
     * @param[in] scan_stop_reason: The stop reason. See @ref gap_stopped_reason_t.
     * @retval void
     ****************************************************************************************
     */     
    void (*app_gap_scan_stop_cb)(uint8_t status, gap_stopped_reason_t scan_stop_reason);

    /**
     ****************************************************************************************
     * @brief This callback function will be called once the advertising report has been received.
     * @param[in] p_adv_report: The extended advertising report. See @ref gap_ext_adv_report_ind_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_gap_adv_report_ind_cb)(const gap_ext_adv_report_ind_t  *p_adv_report);

    /**
     ****************************************************************************************
     * @brief This callback function will be called once the periodic advertising synchronization has been established.
     * @param[in] status:                  The status of sync.
     * @param[in] inst_idx:                The periodic syncronization index. Valid range is: 0 - 4.
     * @param[in] p_sync_established_info: The established ind info.  See @ref gap_sync_established_ind_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_gap_sync_establish_cb)(uint8_t inst_idx, uint8_t status, const gap_sync_established_ind_t *p_sync_established_info);

    /**
     ****************************************************************************************
     * @brief This callback function will be called when sync has stopped.
     * @param[in] status:          The status of stopping sync.
     * @param[in] inst_idx:        The periodic syncronization index. Valid range is: 0 - 4.
     * @retval void
     ****************************************************************************************
     */      
    void (*app_gap_stop_sync_cb)(uint8_t inst_idx, uint8_t status);
        
    /**
     ****************************************************************************************
     * @brief This callback function will be called once the periodic advertising synchronization has been lost.
     * @retval void
     ****************************************************************************************
     */
    void (*app_gap_sync_lost_cb)(uint8_t inst_idx);

    // ----------------------------------------------------------------------------------
    // -------------------------   Initiating Callbacks   --------------------------------------
    // ----------------------------------------------------------------------------------
     
    /**
      ****************************************************************************************
      * @brief This callback function will be called when connection completed.
      * @param[in] conn_idx:     The connection index.
      * @param[in] status:       The status of operation. If status is not success, conn_idx and p_conn_param are invalid.
      * @param[in] p_conn_param: The connection param.  See @ref gap_conn_cmp_t.
      * @retval void
      ****************************************************************************************
      */
    void (*app_gap_connect_cb)(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);

     /**
      ****************************************************************************************
      * @brief This callback function will be called when disconnection completed.
      * @param[in] conn_idx: The connection index.
      * @param[in] status:   The status of operation. If status is not success, disconnection reason is invalid.
      * @param[in] reason:   The reason of disconnection. See @ref BLE_STACK_ERROR_CODES.
      * @retval void
      ****************************************************************************************
      */   
    void (*app_gap_disconnect_cb)(uint8_t conn_idx, uint8_t status, uint8_t reason);
    
    /**
     ****************************************************************************************
     * @brief This callback function will be called when connection canceled.
     * @param[in] status:       The status of cancel operation.
     * @retval void
     ****************************************************************************************
    */
    void (*app_gap_connect_cancel_cb)(uint8_t status);

      /**
      ****************************************************************************************
      * @brief This callback function will be called when an automatic connection timeout occurs.
      * @retval void
      ****************************************************************************************
      */
    void (*app_gap_auto_connection_timeout_cb)(void);

    /**
      ****************************************************************************************
      * @brief This callback function will be called when the peer name info has been available.
      * @param[in] conn_idx:    The connection index.
      * @param[in] p_peer_name: The peer device name indication info. See @ref gap_peer_name_ind_t.
      * @retval void
      ****************************************************************************************
      */  
    void (*app_gap_peer_name_ind_cb)(uint8_t conn_idx, const gap_peer_name_ind_t  *p_peer_name);  


    // ----------------------------------------------------------------------------------
    // -------------------------   Connection Control Callbacks  ------------------------------
    // ----------------------------------------------------------------------------------
    
    /**
      ****************************************************************************************
      * @brief This callback function will be called when connection update completed.
      * @param[in] conn_idx:                 The connection index.
      * @param[in] status:                   The status of GAP operation.
      * @param[in] p_conn_param_update_info: The connection update complete param. See @ref gap_conn_update_cmp_t.
      * @retval void
      ****************************************************************************************
      */   
    void (*app_gap_connection_update_cb)(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info);
    
    /**
      ****************************************************************************************
      *@brief This callback function will be called when the peer device requests updating connection.
      * @param[in] conn_idx:                The connection index.
      * @param[in] p_conn_param_update_req: The connection update request param. See @ref gap_conn_param_t.
      * @retval void
      ****************************************************************************************
      */
    void (*app_gap_connection_update_req_cb)(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req); 
    
    /**
      ****************************************************************************************
      * @brief This callback function will be called when app has got the connection info.
      * @param[in] conn_idx:                The connection index.
      * @param[in] status:                  The status of GAP operation.
      * @param[in] p_conn_param_update_req: The connection info. See @ref  gap_conn_info_param_t.
      * @retval void
      ****************************************************************************************
      */
    void (*app_gap_connection_info_get_cb)(uint8_t conn_idx, uint8_t status, const gap_conn_info_param_t *p_conn_info);
    
    /**
     ****************************************************************************************
     * @brief This callback function will be called when app has got the peer info.
     * @param[in]  conn_idx:        The connection index.
     * @param[in]  status:          The status of GAP operation.
     * @param[in]  p_peer_dev_info: The peer device info. See @ref gap_peer_info_param_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_gap_peer_info_get_cb)(uint8_t conn_idx,  uint8_t status, const gap_peer_info_param_t *p_peer_dev_info); 

    /**
     ****************************************************************************************
     * @brief This callback function will be called when an app sets the length size of the supported data.
     * @param[in]  conn_idx:                     The connection index.
     * @param[in]  status:                       The status of GAP operation.
     * @param[in]  p_supported_data_length_size: Supported data length size. See @ref gap_le_pkt_size_ind_t.
     * @retval void
     ****************************************************************************************
     */    
    void (*app_gap_le_pkt_size_info_cb)(uint8_t conn_idx,  uint8_t status, const gap_le_pkt_size_ind_t *p_supported_data_length_size);

    /**
     ****************************************************************************************
     * @brief This callback function will be called when an app reads the local or peer reslovable address.
     * @param[in]  status:                       The status of GAP operation.
     * @param[in]  p_read_rslv_addr:             Read resolvable address info. See @ref gap_rslv_addr_read_t.
     * @retval void
     ****************************************************************************************
     */    
    void (*app_rslv_addr_read_cb)(uint8_t status, const gap_rslv_addr_read_t *p_read_rslv_addr);

#if defined(GR551xx_D0)
    /**
     ****************************************************************************************
     * @brief This callback function will be called when an connection IQ report received.
     * @param[in]  conn_idx:                     The connection index.
     * @param[in]  p_conn_iq_report:             The connection IQ report info. See @ref gap_conn_iq_report_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_conn_iq_report_cb)(uint8_t conn_idx, const gap_conn_iq_report_t *p_conn_iq_report);

    /**
     ****************************************************************************************
     * @brief This callback function will be called when an connectionless IQ report received.
     * @param[in]  per_sync_idx:                 The index of the periodic syncronization instance.
     * @param[in]  p_connless_iq_report:         The connectionless IQ report info. See @ref gap_connless_iq_report_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_connless_iq_report_cb)(uint8_t per_sync_idx, const gap_connless_iq_report_t *p_connless_iq_report);

    /**
     ****************************************************************************************
     * @brief This callback function will be called an app read the local transmit power.
     * @param[in]  conn_idx:                 The connection index.
     * @param[in]  status:                   The status of GAP operation.
     * @param[in]  p_local_tx_pwr_read:      The local transmit power read info, See @ref gap_local_tx_pwr_read_ind_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_local_tx_power_read_cb)(uint8_t conn_idx, uint8_t status, const gap_local_tx_pwr_read_ind_t *p_local_tx_pwr_read);

     /**
     ****************************************************************************************
     * @brief This callback function will be called when an app read the remote transmit power.
     * @param[in]  conn_idx:                 The connection index.
     * @param[in]  status:                   The status of GAP operation.
     * @param[in]  p_remote_tx_pwr_read:     The remote transmit power read info, See @ref gap_remote_tx_pwr_read_ind_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_remote_tx_power_read_cb)(uint8_t conn_idx, uint8_t status, const gap_remote_tx_pwr_read_ind_t *p_remote_tx_pwr_read);

    /**
     ****************************************************************************************
     * @brief This callback function will be called when an transmit power change reporting event received.
     * @param[in]  conn_idx:                 The connection index.
     * @param[in]  p_tx_pwr_report:          The transmit power report info, See @ref gap_tx_pwr_change_report_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_tx_power_change_report_cb)(uint8_t conn_idx, const gap_tx_pwr_change_report_t *p_tx_pwr_change_report);

    /**
     ****************************************************************************************
     * @brief This callback function will be called when a path loss threshold crossing.
     * @param[in]  conn_idx:                 The connection index.
     * @param[in]  p_path_loss_report:       The path loss threshold report info, See @ref gap_path_loss_threshold_report_t.
     * @retval void
     ****************************************************************************************
     */
    void (*app_path_loss_threshold_report_cb)(uint8_t conn_idx, const gap_path_loss_threshold_report_t *p_path_loss_report);
#endif
}gap_cb_fun_t;

/** @} */

/**
 * @defgroup BLE_GAPC_FUNCTION Functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Terminate an existing connection.
 * @note When APP wants to disconnect a connection, it should call this function, and when the disconnection is finished, 
 *       the callback function @ref gap_cb_fun_t::app_gap_disconnect_cb will be called.
 *
 * @param[in] conn_idx: The index of connection.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
  */
uint16_t ble_gap_disconnect(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief Change the Link Layer connection parameters of a connection. 
 * @note When APP wants to update connection param, it should call this function, and if the update connection param is finished,
 *       the callback function @ref gap_cb_fun_t::app_gap_connection_update_cb will be called.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] p_conn_param: The new connection param.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_param_update (uint8_t conn_idx, const gap_conn_update_param_t *p_conn_param);

/**
 *****************************************************************************************
 * @brief Consult BLE connection activity plan situation function.
 * @note  This function should be called when connection established and no periodic advertising exists.
 *
 * @param[out] p_act_num:        Pointer to the number of existing connection activities.
 * @param[out] p_conn_plan_arr:  Pointer to the global array that stores planned connection activities.
 *                               
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_con_plan_consult(uint8_t *p_act_num, gap_con_plan_tag_t **p_conn_plan_arr);

/**
 ****************************************************************************************
 * @brief Connection param update reply to peer device.
 * 
 * @note If the Host receives the param update request from the peer device, it will call the callback function. 
 *       @ref gap_cb_fun_t::app_gap_connection_update_req_cb, and then the APP should call this function to confirm.
 *
 * @param[in] conn_idx:      The index of connection.
 * @param[in] accept: True to accept connection parameters, false to reject.
 *
 * @retval ::SDK_SUCCESS: Operation is success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_param_update_reply(uint8_t conn_idx, bool accept);

/**
 ****************************************************************************************
 * @brief The suggested maximum transmission packet size and maximum packet transmission time to be used for a given connection.
 * @note Once this operation has been completed, the callback function @ref gap_cb_fun_t::app_gap_le_pkt_size_info_cb will be called.
 *
 * @param[in] conn_idx:   The index of connection.
 * @param[in] tx_octects: Preferred maximum number of payload octets that the local Controller should include in a single Link Layer packet on this connection.
 *            Range 0x001B-0x00FB (all other values reserved for future use).
 * @param[in] tx_time:    Preferred maximum number of microseconds that the local Controller should use to transmit a single Link Layer packet on this connection.
 *            Range 0x0148-0x4290 (all other values reserved for future use).
 *
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_data_length_update(uint8_t conn_idx,  uint16_t  tx_octects , uint16_t tx_time);

/**
 ****************************************************************************************
 * @brief Set the PHY preferences for the connection identified by the connection index.
 * @note Once this operation has been completed, the callback function @ref gap_cb_fun_t::app_gap_phy_update_cb will be called.
 *
 * @param[in] conn_idx:   The index of connection.
 * @param[in] tx_phys: the transmitter PHYs that the Host prefers the Controller to use (see @ref BLE_GAP_PHYS).
 * @param[in] rx_phys: A bit field that indicates the receiver PHYs that the Host prefers the Controller to use (see @ref BLE_GAP_PHYS).
 * @param[in] phy_opt: A bit field that allows the Host to specify options for PHYs (see @ref BLE_GAP_PHY_OPTIONS).
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_phy_update(uint8_t conn_idx, uint8_t tx_phys , uint8_t rx_phys, uint8_t phy_opt);

/**
 ****************************************************************************************
 * @brief Get the information of the connection.
 * @note Once the connection information has been available, the callback function @ref gap_cb_fun_t::app_gap_connection_info_get_cb will be called.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] opcode:   The operation code. See @ref gap_get_conn_info_op_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_info_get(uint8_t conn_idx, gap_get_conn_info_op_t opcode);

/**
 ****************************************************************************************
 * @brief Get the information of the peer device.
 * @note Once the peer information has been available, the callback function @ref gap_cb_fun_t::app_gap_peer_info_get_cb will be called.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] opcode:   The operation code. See @ref gap_get_peer_info_op_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_peer_info_get(uint8_t conn_idx, gap_get_peer_info_op_t opcode);

#if defined(GR551xx_D0)
/**
 ****************************************************************************************
 * @brief Send synchronization information about the periodic advertising in an advertising set to a connected device.
 *
 * @note Need to get the feature of peer device before invoke this function. 
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] per_adv_idx:  The index of per adv.
 * @param[in] service_data: Identify the periodic advertisement to the peer device.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_set_info_trans(uint8_t conn_idx, uint8_t per_adv_idx, uint16_t service_data);

/**
 ****************************************************************************************
 * @brief Send synchronization information about the periodic advertising identified by the sync_hdl parameter to a connected device.
 *
 * @param[in] conn_idx:      The index of connection.
 * @param[in] per_sync_idx:  The index of the periodic syncronization instance.
 * @param[in] service_data:  Identify the periodic advertisement to the peer device.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_per_adv_sync_trans(uint8_t conn_idx, uint8_t per_sync_idx, uint16_t service_data);

/**
 ****************************************************************************************
 * @brief Set connection CTE transmit parameters.
 *
 * @param[in] conn_idx:  The index of connection.
 * @param[in] param:     Set connection CTE transmit parameters info, see @ref gap_set_conn_cte_trans_param_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_trans_param_set(uint8_t conn_idx, gap_set_conn_cte_trans_param_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE receive parameters.
 *
 * @param[in] conn_idx:  The index of connection.
 * @param[in] param:     Set connection CTE receive parameters info, see @ref gap_set_conn_cte_rcv_param_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_recv_param_set(uint8_t conn_idx, gap_set_conn_cte_rcv_param_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE request enable.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  Wheter to request the cte for the connection. If enable_flag is set to false, the param shall be NULL.
 * @param[in] param:        Set connection CTE request enable info, see @ref gap_set_conn_cte_req_enable_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_req_enable_set(uint8_t conn_idx, bool enable_flag, gap_set_conn_cte_req_enable_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE response enable.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  Wheter to response the cte req for the connection.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_rsp_enable_set(uint8_t conn_idx, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Read the local current and maximum transmit power levels for the connection identified by the conn_idx.
 * @note Once the local transmit power level has been available, the callback function @ref gap_cb_fun_t::app_local_tx_power_read_cb will be called.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] phy:          Read the transmit power levels on which phy, see @ref gap_phy_type_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_local_tx_pwr_level_read(uint8_t conn_idx, gap_phy_type_t phy);

/**
 ****************************************************************************************
 * @brief Read the remote transmit power levels for the connection identified by the conn_idx.
 * @note Once the remote transmit power level has been available, the callback function @ref gap_cb_fun_t::app_remote_tx_power_read_cb will be called.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] phy:          Read the transmit power levels on which phy, see @ref gap_phy_type_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_remote_tx_pwr_level_read(uint8_t conn_idx, gap_phy_type_t phy);

/**
 ****************************************************************************************
 * @brief Set the path loss threshold reporting parameters for the connection identified by the conn_idx.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] param:        Set path loss report parameter, see @ref gap_set_path_loss_report_param_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_path_loss_report_parameter_set(uint8_t conn_idx, gap_set_path_loss_report_param_t *param);

/**
 ****************************************************************************************
 * @brief Enable or disable path loss reporting for the connection identified by the conn_idx.
 * @note Once a path loss threshold crossing, the callback function @ref gap_cb_fun_t::app_path_loss_threshold_report_cb will be called.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  The enable flag for reporting path loss.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_path_loss_report_enable_set(uint8_t conn_idx, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Enable or disable the reporting of transmit power level changes in the local and remote for the connection identified by the conn_idx.
 * @note Once the transmit power changes, the callback function @ref gap_cb_fun_t::app_tx_power_change_report_cb will be called.
 *
 * @param[in] conn_idx:           The index of connection.
 * @param[in] local_enable_flag:  The enable flag for reporting transmit power level changes in the local.
 * @param[in] remote_enable_flag: The enable flag for reporting transmit power level changes in the remote.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_tx_pwr_change_report_enable_set(uint8_t conn_idx, bool local_enable_flag, bool remote_enable_flag);

#endif
/** @} */

#endif
/** @} */
/** @} */
/** @} */
