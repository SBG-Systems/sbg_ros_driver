#ifndef CONFIG_APPLIER_H
#define CONFIG_APPLIER_H

// Standard headers
#include <array>
#include <string>

// Project headers
#include <config_store.h>

namespace sbg
{
/*!
 * Class to apply configuration to a device.
 */
class ConfigApplier
{
private:

  bool m_reboot_needed_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Check if the configuration getter worked properly.
   * This function will only log a warning information if the getter did not work.
   * 
   * \param[in] ref_sbg_error_code          Error code from the configuration getter.
   * \param[in] ref_conf_title              String to identify the configuration.
   */
  void checkConfigurationGet(const SbgErrorCode& ref_sbg_error_code, const std::string& ref_conf_title) const;

  /*!
   * Check if the configuration has been applied correctly.
   * This function will log if the configuration has been applied.
   * If it did not work, a warning log will be displayed, but the application will keep running.
   * It will be the user responsability to check and decide.
   * 
   * \param[in] ref_sbg_error_code          Error code from the configuration getter.
   * \param[in] ref_conf_title              String to identify the configuration.
   */
  void checkConfigurationApplied(const SbgErrorCode& ref_sbg_error_code, const std::string& ref_conf_title);

  /*!
   * Configure the initial condition parameters.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_init_condition          Initial condition conf to apply.
   */
  void configureInitCondition(SbgEComHandle& ref_sbg_com_handle, const SbgEComInitConditionConf& ref_init_condition);

  /*!
   * Configure the motion profile.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_motion_profile          Motion profile configuration to apply.
   */
  void configureMotionProfile(SbgEComHandle& ref_sbg_com_handle, const SbgEComModelInfo& ref_motion_profile);

  /*!
   * Configure the IMU alignement.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_sensor_align            Sensor IMU alignement configuration to apply.
   * \param[in] level_arms                  X, Y, Z level arms to apply.
   */
  void configureImuAlignement(SbgEComHandle& ref_sbg_com_handle, const SbgEComSensorAlignmentInfo& ref_sensor_align, std::array<float, 3> level_arms);

  /*!
   * Configure the aiding assignement.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_aiding_assign           Aiding assignement configuration to apply.
   */
  void configureAidingAssignement(SbgEComHandle& ref_sbg_com_handle, const SbgEComAidingAssignConf& ref_aiding_assign);

  /*!
   * Configure the magnetometers model.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_mag_model               Magnetometers model configuration to apply.
   */
  void configureMagModel(SbgEComHandle& ref_sbg_com_handle, const SbgEComModelInfo& ref_mag_model);

  /*!
   * Configure the magnetometers rejection.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_mag_rejection           Magnetometers rejection configuration to apply.
   */
  void configureMagRejection(SbgEComHandle& ref_sbg_com_handle, const SbgEComMagRejectionConf& ref_mag_rejection);

  /*!
   * Configure the Gnss model.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_gnss_model              Gnss model configuration to apply.
   */
  void configureGnssModel(SbgEComHandle& ref_sbg_com_handle, const SbgEComModelInfo& ref_gnss_model);

  /*!
   * Configure the Gnss alignement.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_gnss_alignement         Gnss alignement configuration to apply.
   */
  void configureGnssAlignement(SbgEComHandle& ref_sbg_com_handle, const SbgEComGnssAlignmentInfo& ref_gnss_alignement);

  /*!
   * Configure the Gnss rejection.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_gnss_rejection          Gnss rejection configuration to apply.
   */
  void configureGnssRejection(SbgEComHandle& ref_sbg_com_handle, const SbgEComGnssRejectionConf& ref_gnss_rejection);

  /*!
   * Configure the odometer.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_odometer                Odometer configuration to apply.
   */
  void configureOdometer(SbgEComHandle& ref_sbg_com_handle, const SbgEComOdoConf& ref_odometer);

  /*!
   * Configure the odometer level arm.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] odometer_level_arms         X,Y,Z odometer level arms to apply.
   */
  void configureOdometerLevelArm(SbgEComHandle& ref_sbg_com_handle, std::array<float, 3> odometer_level_arms);

  /*!
   * Configure the odometer rejection.
   * 
   * \param[in] ref_sbg_com_handle          SBG communication handle.
   * \param[in] ref_odometer_rejection      Odometer rejection configuration to apply.
   */
  void configureOdometerRejection(SbgEComHandle& ref_sbg_com_handle, const SbgEComOdoRejectionConf& ref_odometer_rejection);

  /*!
   * Configure the output for the SBG log.
   *
   * \param[in] ref_com_handle    SBG communication handle.
   * \param[in] output_port       Output communication port.
   * \param[in] ref_log_output    Log output to configure.
   */
  void configureOutput(SbgEComHandle& ref_sbg_com_handle, SbgEComOutputPort output_port, const ConfigStore::SbgLogOutput &ref_log_output);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   */
  ConfigApplier(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Apply a configuration to the SBG device.
   * 
   * \param[in] ref_config_store            Configuration to apply.
   * \param[in] ref_sbg_com_handle          SbgECom handle to apply the configuration to the device.
   */
  void applyConfiguration(const ConfigStore& ref_config_store, SbgEComHandle& ref_sbg_com_handle);

  /*!
   * Save the configuration to the device.
   * 
   * \param[in] ref_sbg_com_handle          SbgECom handle to apply the configuration to the device.
   */
  void saveConfiguration(SbgEComHandle& ref_sbg_com_handle);
};
}

#endif // CONFIG_APPLIER_H