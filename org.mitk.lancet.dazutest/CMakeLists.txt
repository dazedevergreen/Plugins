project(org_mitk_lancet_dazutest)

mitk_create_plugin(
  EXPORT_DIRECTIVE DAZUTEST_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkIGTUI MitkLancetIGT MitkLancetRobot MitkLancetAlgo MitkLancetHardwareDevice MitkLancetRobot MitkLancetRobotRegistration
)
