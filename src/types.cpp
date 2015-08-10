#include "types.h"


namespace visual_odometry {


/********** FEATURE ************/

// Static member
int Feature::next_id_s_ = 0;



/********** MEMORY ************/


// Receives other memory, makes data association
//and returns matched features ready to use for pose estimation (memmap_out)
void Memory::add(const Memory& other, MemMap& memmap_out){

    // Match features of *this and other => (output) Matches
    // Update type of matched features in *this and add the new ones
    // Filter and returns Circle features (memmap_out)




}

void Memory::update(const Memory& other){

}



}
