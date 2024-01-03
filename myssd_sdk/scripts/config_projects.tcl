set PLATFORM_NAME zu_onfi8
set SYS_PROJ_NAME nvme_ftl_system
set FTL_PROJ_NAME nvme_ftl
set FIL_PROJ_NAME nvme_fil
set ECC_PROJ_NAME nvme_ecc

app config -name $FTL_PROJ_NAME -add include-path "\${workspace_loc:/$FTL_PROJ_NAME/src}"
app config -name $FTL_PROJ_NAME -add include-path "\${workspace_loc:/$FTL_PROJ_NAME/include}"
app config -name $FTL_PROJ_NAME -add libraries m

# Create FIL project
app config -name $FIL_PROJ_NAME -add include-path "\${workspace_loc:/$FIL_PROJ_NAME/src}"
app config -name $FIL_PROJ_NAME -add include-path "\${workspace_loc:/$FTL_PROJ_NAME/include}"

# Create ECC project
app config -name $ECC_PROJ_NAME -add include-path "\${workspace_loc:/$ECC_PROJ_NAME/src}"
app config -name $ECC_PROJ_NAME -add include-path "\${workspace_loc:/$FTL_PROJ_NAME/include}"

