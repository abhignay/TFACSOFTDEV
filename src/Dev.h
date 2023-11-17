// Test function to check if fc @ 10m

uint32_t* Pre10meterMillis;

void checkFor10(float alt, float state){
    if (alt >= 13000){
        state++;
        Pre10meterMillis = millis() + 10000;
    }
}