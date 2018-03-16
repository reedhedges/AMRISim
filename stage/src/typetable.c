

#include "stage_internal.h"

// declare the initialization functions for specialized models

#ifdef ENABLE_BLOBFINDER_MODEL
int blobfinder_init( stg_model_t* mod );
#endif

#ifdef ENABLE_FIDUCIAL_MODEL
int fiducial_init( stg_model_t* mod );
#endif

#ifdef ENABLE_GRIPPER_MODEL
int gripper_init( stg_model_t* mod );
#endif

int laser_init( stg_model_t* mod );
int position_init( stg_model_t* mod );
int ranger_init( stg_model_t* mod );
int messages_init( stg_model_t* mod );

// map worldfile keywords onto initialization functions
stg_type_record_t typetable[] = 
  {    
    { "position", position_init },
    { "ranger",  ranger_init },
    { "laser", laser_init },
    { "sonar", ranger_init },

#ifdef ENABLE_BLOBFINDER_MODEL
    { "blobfinder",blobfinder_init },
#endif

#ifdef ENABLE_FIDUCIAL_MODEL
    { "fiducialfinder", fiducial_init },
#endif

#ifdef ENABLE_GRIPPER_MODEL
    { "gripper", gripper_init },      
#endif

    { "messages", messages_init },
    { NULL, NULL } // this must be the last entry
  };

stg_type_record_t* stg_user_typetable = NULL;  // Custom types added by stg_register_custom_type()

