/******************************************************************************/
/*                 GR55xx BLE Peripherals Interrupt Handlers                  */
/*  Add here the Interrupt Handler for the BLE peripheral(s), for the         */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_gr55xx.s).                                                  */
/*  Note: The Application project should not contain this file when sdk.lib   */
/*        be linked.                                                          */
/******************************************************************************/

/**
  * @brief  This function handles SVC Handler.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
    while(1);
}

/**
  * @brief  This function handles BLESDK Handler.
  * @param  None
  * @retval None
  */
void BLE_SDK_Handler(void)
{
    while(1);
}

/**
  * @brief  This function handles BLE Handler.
  * @param  None
  * @retval None
  */
void BLE_IRQHandler(void)
{
    while(1);
}

/**
  * @brief  This function handles BLESLP Handler.
  * @param  None
  * @retval None
  */
void BLESLP_IRQHandler(void)
{
    while(1);
}
