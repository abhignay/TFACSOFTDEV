// state machinr threshhold and time
int launchthresh = 2; // launch detect in G's
int descentthresh = 1.5; // 2 meters
int chutethresh = 100; // we want to be 100 meters above ground level to deploy the chutes
uint32_t fire_length = 5000; // pyro will fire for 1.5 seconds
int landedthresh = 2; // to check whether we have landed we should be under 5 meters