/**
 * BLE communication module for ENAV
 * This file handles all BLE communication with the ESP32
 */

let bleDevice;
let bleServer;
let bleServiceUuid = "0000ffe0-0000-1000-8000-00805f9b34fb";
let txCharacteristicUuid = "0000ffe1-0000-1000-8000-00805f9b34fb";
let rxCharacteristicUuid = "0000ffe2-0000-1000-8000-00805f9b34fb";
let txCharacteristic;
let rxCharacteristic;
let bleDataCallback = null;

// Register a callback function to be called when data is received from BLE
function registerBLEDataCallback(callback) {
    bleDataCallback = callback;
    console.log("BLE data callback registered");
}

let map, marker;
let isFirstPOIUpdate = true; // Track if this is the first POI update

// Update the connection status icon
function updateConnectionStatus(connected) {
    const connectionStatus = document.getElementById('connectionIcon');
    if (!connectionStatus) {
        console.error('Connection status icon element not found in the DOM.');
        return;
    }

    if (connected) {
        connectionStatus.src = 'connected.png'; // Icon for connected state
        connectionStatus.alt = 'Connected';
    } else {
        connectionStatus.src = 'disconnected.png'; // Icon for disconnected state
        connectionStatus.alt = 'Disconnected';
    }
}

// Ensure BLE initialization and connection handling are robust
async function connectToBLE() {
    try {
        console.log('Requesting BLE device...');
        bleDevice = await navigator.bluetooth.requestDevice({
            filters: [{ namePrefix: 'ENAV' }], // Filter by device name prefix
            optionalServices: [bleServiceUuid]
        });

        console.log('Connecting to GATT server...');
        bleServer = await bleDevice.gatt.connect();

        console.log('Getting service...');
        const service = await bleServer.getPrimaryService(bleServiceUuid);

        console.log('Getting characteristics...');
        txCharacteristic = await service.getCharacteristic(txCharacteristicUuid);
        rxCharacteristic = await service.getCharacteristic(rxCharacteristicUuid);

        // Enable notifications for receiving data
        await rxCharacteristic.startNotifications();
        rxCharacteristic.addEventListener('characteristicvaluechanged', handleBleData);

        console.log('Connected to BLE device!');

        // Update connection status
        updateConnectionStatus(true);
        if (bleDataCallback) {
            bleDataCallback({ connected: true });
        }

        // Fetch initial data
        await sendCommandToESP32('GET_DATA');

        // Monitor connection
        bleDevice.addEventListener('gattserverdisconnected', handleDisconnection);

        return true;
    } catch (error) {
        console.error('BLE connection error:', error);
        alert('Failed to connect to BLE device: ' + error.message);
        updateConnectionStatus(false);
        return false;
    }
}

// Ensure disconnection handling is robust
function handleDisconnection() {
    console.log('Device disconnected');

    if (bleDataCallback) {
        bleDataCallback({ connected: false });
    }

    // Clear variables
    txCharacteristic = null;
    rxCharacteristic = null;
    bleServer = null;

    // Restart advertising or allow reconnection
    alert('BLE device disconnected. Please reconnect.');
}

// Send a command to the ESP32
async function sendCommandToESP32(command) {
    if (!txCharacteristic) {
        console.log('No txCharacteristic available. Attempting to connect...');
        const connected = await connectToBLE();
        if (!connected) {
            return false;
        }
    }
    
    console.log('Sending command:', command);
    
    try {
        const encoder = new TextEncoder();
        await txCharacteristic.writeValue(encoder.encode(command));
        console.log('Command sent successfully');
        return true;
    } catch (error) {
        console.error('Error sending command:', error);
        alert('Failed to send command: ' + error);
        return false;
    }
}

// Handle data received from the ESP32
function handleBleData(event) {
    const decoder = new TextDecoder();
    const data = decoder.decode(event.target.value);
    console.log('Received BLE data:', data);
    
    // Parse the data string
    try {
        const parsedData = {};
        parsedData.connected = true; // Mark as connected since we received data
        
        // Extract home coordinates
        const homeMatch = data.match(/Home Lat: ([-\d.]+), Lon: ([-\d.]+)/);
        if (homeMatch) {
            parsedData.homeLat = homeMatch[1];
            parsedData.homeLon = homeMatch[2];
            document.getElementById('savedHomeLat').value = homeMatch[1];
            document.getElementById('savedHomeLon').value = homeMatch[2];
            console.log(`Extracted home coordinates: ${parsedData.homeLat}, ${parsedData.homeLon}`);
        }

        // Extract fuel information
        const fuelMatch = data.match(/Fuel: ([\d.]+), Burn Rate: ([\d.]+)/);
        if (fuelMatch) {
            document.getElementById('newFuelLevel').value = fuelMatch[1];
            document.getElementById('newBurnRate').value = fuelMatch[2];
            console.log(`Extracted fuel data: Level=${fuelMatch[1]}L, Rate=${fuelMatch[2]}L/h`);
        }

        // Extract POI information
        for (let i = 1; i <= 3; i++) {
            const poiRegex = new RegExp(`POI${i}: Lat=([\\-\\d\\.]+), Lon=([\\-\\d\\.]+), En=(\\d)`);
            const poiMatch = data.match(poiRegex);
            if (poiMatch) {
                document.getElementById(`poi${i}Lat`).value = poiMatch[1];
                document.getElementById(`poi${i}Lon`).value = poiMatch[2];
                document.getElementById(`poi${i}Enabled`).checked = poiMatch[3] === '1';
                console.log(`Extracted POI ${i}: Lat=${poiMatch[1]}, Lon=${poiMatch[2]}, Enabled=${poiMatch[3]}`);
            }
        }

        // Call the callback with the parsed data
        if (bleDataCallback) {
            bleDataCallback(parsedData);
        }
    } catch (error) {
        console.error('Error parsing BLE data:', error);
    }
}

// Fetch data from the device
async function fetchDataFromDevice() {
    try {
        console.log('Requesting data from device...');
        await sendCommandToESP32('GET_DATA');
        return true;
    } catch (error) {
        console.error('Error fetching data from device:', error);
        return false;
    }
}

// Add a fallback for unsupported browsers
document.addEventListener('DOMContentLoaded', () => {
    if (!navigator.bluetooth) {
        alert('Your browser does not support Web Bluetooth. Please use a compatible browser like Chrome.');
        return;
    }

    document.getElementById('connectButton').addEventListener('click', async () => {
        const connected = await connectToBLE();
        if (connected) {
            alert('Successfully connected to BLE device!');
        } else {
            alert('Failed to connect to BLE device. Please try again.');
        }
    });

    document.getElementById('updatePOIButton').addEventListener('click', async () => {
        const lat = parseFloat(document.getElementById('poiLat').value);
        const lon = parseFloat(document.getElementById('poiLon').value);
        const enabled = document.getElementById('poiEnabled').checked ? 1 : 0;
        
        if (!isNaN(lat) && !isNaN(lon)) {
            console.log(`Sending POI update: Lat=${lat}, Lon=${lon}, Enabled=${enabled}`);
            const command = `${lat}:${lon}:${enabled}`;
            const success = await sendCommandToESP32(command);
            
            if (success) {
                alert('POI updated successfully!');
            }
        } else {
            alert('Please enter valid latitude and longitude');
        }
    });
    
    document.getElementById('updateFuelButton').addEventListener('click', async () => {
        const fuelLevel = parseFloat(document.getElementById('newFuelLevel').value);
        const burnRate = parseFloat(document.getElementById('newBurnRate').value);
        
        if (!isNaN(fuelLevel) && !isNaN(burnRate) && fuelLevel > 0 && burnRate > 0) {
            console.log(`Sending fuel update: Level=${fuelLevel}L, Rate=${burnRate}L/h`);
            const command = `FUEL:${fuelLevel}:${burnRate}`;
            const success = await sendCommandToESP32(command);
            
            if (success) {
                console.log("Fuel data sent successfully via BLE");
            }
        } else {
            alert('Please enter valid numbers for both fuel level and burn rate');
        }
    });
});

// Make functions available globally
window.connectToBLE = connectToBLE;
window.sendCommandToESP32 = sendCommandToESP32;
window.registerBLEDataCallback = registerBLEDataCallback;
