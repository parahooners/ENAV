// Function to set POI
function setPOI(index, latitude, longitude, enabled) {
    const command = `POI:${index}:${latitude}:${longitude}:${enabled ? 1 : 0}`;
    console.log(`Setting POI: ${command}`); // Debug log
    sendBLECommand(command);
}

// Function to update all POIs
function updateAllPOIs(pois) {
    pois.forEach((poi, index) => {
        const { latitude, longitude, enabled } = poi;
        setPOI(index + 1, latitude, longitude, enabled); // Use 1-based index
    });
}

// Function to set mode (walking or flying)
function setMode(mode) {
    const command = `MODE:${mode}`; // 1 for flying, 2 for walking
    console.log(`Setting mode: ${command}`); // Debug log
    sendBLECommand(command);
}

// Function to set fuel data
function setFuelData(level, rate) {
    const command = `FUEL:${level}:${rate}`;
    sendBLECommand(command);
}

// Function to request data
function getData() {
    const command = "GET_DATA";
    sendBLECommand(command);
}

// Function to send BLE command
function sendBLECommand(command) {
    console.log(`Sending BLE command: ${command}`);
    // Replace this with the actual BLE communication logic
    // Example: bleCharacteristic.writeValue(command);
}
