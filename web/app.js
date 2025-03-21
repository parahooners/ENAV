/**
 * Main application script for ENAV Manager
 * Handles map, BLE connectivity, and POI management
 */

// BLE Service and Characteristic UUIDs
const bleServiceUuid = "0000ffe0-0000-1000-8000-00805f9b34fb";
const rxCharacteristicUuid = "0000ffe1-0000-1000-8000-00805f9b34fb"; 
const txCharacteristicUuid = "0000ffe2-0000-1000-8000-00805f9b34fb";

// Global variables
let bleDevice, bleServer;
let rxCharacteristic, txCharacteristic;
let mainMap, homeMarker;
let poiMarkers = [];
let poiColors = ['red', 'blue', 'green'];
let activeSetPOIIndex = -1;

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    initializeMap();
    setupEventListeners();
    
    // Initialize POI status indicators
    for (let i = 1; i <= 3; i++) {
        updatePOIStatusIndicator(i);
    }
});

// Initialize the map with all necessary components
function initializeMap() {
    // Create main map centered at a default position
    mainMap = L.map('map').setView([51.505, -0.09], 13);
    
    // Add tile layer (map imagery)
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(mainMap);
    
    // Create home marker
    homeMarker = L.marker([51.505, -0.09], { 
        icon: createIcon('yellow'),
        draggable: false
    });
    homeMarker.bindPopup("Home Location");
    homeMarker.addTo(mainMap);
    
    // Create POI markers
    for (let i = 0; i < 3; i++) {
        const marker = L.marker([0, 0], {
            icon: createIcon(poiColors[i]),
            draggable: true
        });
        
        // When marker is dragged, update corresponding input fields
        marker.on('dragend', function() {
            const position = marker.getLatLng();
            document.getElementById(`poi${i+1}Lat`).value = position.lat.toFixed(6);
            document.getElementById(`poi${i+1}Lon`).value = position.lng.toFixed(6);
            updatePOIStatusIndicator(i+1);
        });
        
        // Add popup with POI info
        marker.bindPopup(`POI ${i+1}`);
        poiMarkers.push(marker);
        // Don't add to map yet - will add when enabled
    }
    
    // Add click handler to set POIs
    mainMap.on('click', function(e) {
        if (activeSetPOIIndex >= 0) {
            const i = activeSetPOIIndex + 1;
            document.getElementById(`poi${i}Lat`).value = e.latlng.lat.toFixed(6);
            document.getElementById(`poi${i}Lon`).value = e.latlng.lng.toFixed(6);
            poiMarkers[activeSetPOIIndex].setLatLng(e.latlng);
            
            // Ensure the marker is on the map
            if (!mainMap.hasLayer(poiMarkers[activeSetPOIIndex])) {
                poiMarkers[activeSetPOIIndex].addTo(mainMap);
            }
            
            // After setting location, automatically uncheck the "Set" checkbox
            document.getElementById(`poi${i}SetNew`).checked = false;
            
            // New code: Automatically update the POI on the device
            // Make sure the POI is enabled
            document.getElementById(`poi${i}Enabled`).checked = true;
            updatePOIStatusIndicator(i);
            updateDisableButtonState(i);
            
            // Send the POI update to the device immediately
            updateSinglePOI(i);
            
            activeSetPOIIndex = -1;
        }
    });
    
    // Add map resize handler to ensure it displays correctly
    setTimeout(() => {
        mainMap.invalidateSize();
    }, 100);
    
    // Map should recalculate size if the window is resized
    window.addEventListener('resize', () => {
        mainMap.invalidateSize();
    });
}

// Create custom colored icons for markers
function createIcon(color) {
    return L.icon({
        iconUrl: `https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-${color}.png`,
        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
        iconSize: [25, 41],
        iconAnchor: [12, 41],
        popupAnchor: [1, -34],
        shadowSize: [41, 41]
    });
}

// Setup all event listeners
function setupEventListeners() {
    // BLE Connect button
    document.getElementById('connectButton').addEventListener('click', connectToBLE);
    
    // Update POIs button
    document.getElementById('updatePOIButton').addEventListener('click', updateAllPOIs);
    
    // Update Fuel button
    document.getElementById('updateFuelButton').addEventListener('click', updateFuel);
    
    // Mode selection buttons
    document.getElementById('flyingModeBtn').addEventListener('click', () => setMode(1));
    document.getElementById('walkingModeBtn').addEventListener('click', () => setMode(2));
    
    // Set POI checkboxes and disable buttons
    for (let i = 1; i <= 3; i++) {
        // Enabled checkbox toggles marker visibility
        document.getElementById(`poi${i}Enabled`).addEventListener('change', function(e) {
            const index = i - 1;
            
            // Show/hide marker based on checkbox state
            if (this.checked) {
                showPOIMarker(index);
            } else {
                hidePOIMarker(index);
            }
            
            // Update status indicator
            updatePOIStatusIndicator(i);
            
            // Update disable button state
            updateDisableButtonState(i);
            
            // Prevent default behavior that might cause unexpected unticking
            e.stopPropagation();
        });
        
        // Set New Location checkbox
        document.getElementById(`poi${i}SetNew`).addEventListener('change', function(e) {
            const index = i - 1;
            
            if (this.checked) {
                // Uncheck all other "Set New" checkboxes
                for (let j = 1; j <= 3; j++) {
                    if (j !== i) {
                        document.getElementById(`poi${j}SetNew`).checked = false;
                    }
                }
                
                // Set active POI index
                activeSetPOIIndex = index;
                
                // Ensure marker is visible
                showPOIMarker(index);
                
                // Alert user about action
                alert(`Click on the map to set location for POI ${i}`);
                
                // Prevent event propagation that might cause unticking
                e.stopPropagation();
            } else {
                // If this checkbox was unchecked, reset activeSetPOIIndex
                if (activeSetPOIIndex === index) {
                    activeSetPOIIndex = -1;
                }
            }
        });
        
        // Update marker when inputs change
        document.getElementById(`poi${i}Lat`).addEventListener('change', () => updateMarkerFromInputs(i));
        document.getElementById(`poi${i}Lon`).addEventListener('change', () => updateMarkerFromInputs(i));
        
        // Add event listener for disable button
        document.getElementById(`poi${i}DisableBtn`).addEventListener('click', () => disablePOI(i));
        
        // Initialize disable button state
        updateDisableButtonState(i);
    }
}

// Show POI marker on map
function showPOIMarker(index) {
    const lat = parseFloat(document.getElementById(`poi${index+1}Lat`).value);
    const lon = parseFloat(document.getElementById(`poi${index+1}Lon`).value);
    
    if (!isNaN(lat) && !isNaN(lon)) {
        poiMarkers[index].setLatLng([lat, lon]);
        poiMarkers[index].addTo(mainMap);
    } else {
        // Use map center if no coordinates exist yet
        const center = mainMap.getCenter();
        poiMarkers[index].setLatLng(center);
        poiMarkers[index].addTo(mainMap);
    }
}

// Hide POI marker
function hidePOIMarker(index) {
    if (mainMap.hasLayer(poiMarkers[index])) {
        mainMap.removeLayer(poiMarkers[index]);
    }
}

// Update marker position from input fields
function updateMarkerFromInputs(poiNum) {
    const index = poiNum - 1;
    const lat = parseFloat(document.getElementById(`poi${poiNum}Lat`).value);
    const lon = parseFloat(document.getElementById(`poi${poiNum}Lon`).value);
    
    if (!isNaN(lat) && !isNaN(lon)) {
        poiMarkers[index].setLatLng([lat, lon]);
        
        // If POI is enabled, make sure marker is on map
        if (document.getElementById(`poi${poiNum}Enabled`).checked) {
            poiMarkers[index].addTo(mainMap);
        }
    }
    
    // Update the POI status indicator
    updatePOIStatusIndicator(poiNum);
}

// Add function to update POI status indicator
function updatePOIStatusIndicator(poiNum) {
    const enabled = document.getElementById(`poi${poiNum}Enabled`).checked;
    const statusElement = document.getElementById(`poi${poiNum}Status`);
    
    if (statusElement) {
        statusElement.textContent = enabled ? 'Enabled' : 'Disabled';
        statusElement.className = 'poi-status ' + (enabled ? 'poi-status-enabled' : 'poi-status-disabled');
    }
}

// Connect to BLE device
async function connectToBLE() {
    try {
        console.log('Requesting BLE device...');
        bleDevice = await navigator.bluetooth.requestDevice({
            acceptAllDevices: true,
            optionalServices: [bleServiceUuid]
        });
        
        console.log('Connecting to GATT server...');
        bleServer = await bleDevice.gatt.connect();
        
        console.log('Getting service...');
        const service = await bleServer.getPrimaryService(bleServiceUuid);
        
        console.log('Getting characteristics...');
        rxCharacteristic = await service.getCharacteristic(rxCharacteristicUuid);
        txCharacteristic = await service.getCharacteristic(txCharacteristicUuid);
        
        // Enable notifications
        await txCharacteristic.startNotifications();
        txCharacteristic.addEventListener('characteristicvaluechanged', handleIncomingBLEData);
        
        // Update UI and fetch initial data
        updateConnectionStatus(true);
        console.log('Connected to BLE device!');
        
        // Request data from device
        await sendCommand('GET_DATA');
        
        // Setup disconnect listener
        bleDevice.addEventListener('gattserverdisconnected', handleDisconnection);
        
        return true;
    } catch (error) {
        console.error('BLE connection error:', error);
        alert(`Failed to connect: ${error.message}`);
        updateConnectionStatus(false);
        return false;
    }
}

// Handle disconnection from BLE device
function handleDisconnection() {
    console.log('BLE device disconnected');
    updateConnectionStatus(false);
    
    // Clear BLE variables
    rxCharacteristic = null;
    txCharacteristic = null;
    bleServer = null;
}

// Update UI connection status
function updateConnectionStatus(connected) {
    const iconElem = document.getElementById('connectionIcon');
    const textElem = document.getElementById('connectionText');
    
    if (connected) {
        iconElem.src = 'connected.png';
        iconElem.alt = 'Connected';
        textElem.textContent = 'Connected';
        document.getElementById('connectButton').disabled = true;
    } else {
        iconElem.src = 'disconnected.png';
        iconElem.alt = 'Disconnected';
        textElem.textContent = 'Disconnected';
        document.getElementById('connectButton').disabled = false;
    }
}

// Send command to BLE device
async function sendCommand(command) {
    if (!rxCharacteristic) {
        console.error('Not connected to BLE device');
        return false;
    }
    
    try {
        console.log('Sending command:', command);
        const encoder = new TextEncoder();
        await rxCharacteristic.writeValue(encoder.encode(command));
        return true;
    } catch (error) {
        console.error('Error sending command:', error);
        return false;
    }
}

// Handle incoming BLE data
function handleIncomingBLEData(event) {
    const decoder = new TextDecoder();
    const data = decoder.decode(event.target.value);
    console.log('Received BLE data:', data);
    
    // Parse the data
    try {
        // Extract home coordinates
        const homeMatch = data.match(/Home Lat: ([-\d.]+), Lon: ([-\d.]+)/);
        if (homeMatch) {
            const lat = parseFloat(homeMatch[1]);
            const lon = parseFloat(homeMatch[2]);
            
            // Update home marker and form fields
            if (!isNaN(lat) && !isNaN(lon)) {
                homeMarker.setLatLng([lat, lon]);
                document.getElementById('savedHomeLat').value = lat.toFixed(6);
                document.getElementById('savedHomeLon').value = lon.toFixed(6);
                
                // Center map on home location if this is first data
                if (mainMap.getZoom() === 13) { // Default zoom
                    mainMap.setView([lat, lon], 13);
                }
            }
        }
        
        // Extract POI information - but only if we're not actively setting one
        if (activeSetPOIIndex === -1) {
            for (let i = 1; i <= 3; i++) {
                const poiRegex = new RegExp(`POI${i}: Lat=([\\-\\d\\.]+), Lon=([\\-\\d\\.]+), En=(\\d)`);
                const poiMatch = data.match(poiRegex);
                if (poiMatch) {
                    const lat = parseFloat(poiMatch[1]);
                    const lon = parseFloat(poiMatch[2]);
                    const enabled = poiMatch[3] === '1';
                    
                    // Get current state to avoid unnecessary updates
                    const currentEnabled = document.getElementById(`poi${i}Enabled`).checked;
                    
                    // Update the fields
                    document.getElementById(`poi${i}Lat`).value = lat.toFixed(6);
                    document.getElementById(`poi${i}Lon`).value = lon.toFixed(6);
                    
                    // Only update enabled state if it's different, to avoid unticking
                    if (currentEnabled !== enabled) {
                        document.getElementById(`poi${i}Enabled`).checked = enabled;
                        updatePOIStatusIndicator(i);
                        updateDisableButtonState(i);
                    }
                    
                    // Update marker
                    if (enabled) {
                        poiMarkers[i-1].setLatLng([lat, lon]);
                        poiMarkers[i-1].addTo(mainMap);
                    } else if (!currentEnabled) { // Only remove if it was already disabled
                        if (mainMap.hasLayer(poiMarkers[i-1])) {
                            mainMap.removeLayer(poiMarkers[i-1]);
                        }
                    }
                }
            }
        }
        
        // Extract fuel information
        const fuelMatch = data.match(/Fuel: ([\d.]+), Burn Rate: ([\d.]+)/);
        if (fuelMatch) {
            const fuelLevel = parseFloat(fuelMatch[1]);
            const burnRate = parseFloat(fuelMatch[2]);
            
            document.getElementById('currentFuelLevel').textContent = fuelLevel.toFixed(1);
            document.getElementById('currentBurnRate').textContent = burnRate.toFixed(1);
            
            // Set default values for the input fields if they're empty
            if (document.getElementById('newFuelLevel').value === '') {
                document.getElementById('newFuelLevel').value = fuelLevel.toFixed(1);
            }
            if (document.getElementById('newBurnRate').value === '') {
                document.getElementById('newBurnRate').value = burnRate.toFixed(1);
            }
        }
        
        // Extract operation mode
        const modeMatch = data.match(/Mode: (\d)/);
        if (modeMatch) {
            const mode = parseInt(modeMatch[1]);
            updateModeButtons(mode);
        }
        
        // Extract battery voltage
        const battMatch = data.match(/Batt: ([\d.]+)V/);
        if (battMatch) {
            const voltage = parseFloat(battMatch[1]);
            document.getElementById('batteryVoltage').textContent = voltage.toFixed(2) + 'V';
            
            // Update battery level indicator
            // Assuming voltage range 3.3V (0%) to 4.2V (100%)
            const percentage = Math.max(0, Math.min(100, ((voltage - 3.3) / 0.9) * 100));
            document.getElementById('batteryLevel').style.width = percentage + '%';
            
            // Change color based on level
            if (percentage < 20) {
                document.getElementById('batteryLevel').style.backgroundColor = '#f44336'; // Red
            } else if (percentage < 50) {
                document.getElementById('batteryLevel').style.backgroundColor = '#ff9800'; // Orange
            } else {
                document.getElementById('batteryLevel').style.backgroundColor = '#4CAF50'; // Green
            }
        }
        
    } catch (error) {
        console.error('Error parsing BLE data:', error);
    }
}

// New function to update a single POI
async function updateSinglePOI(poiNum) {
    if (!rxCharacteristic) {
        console.error('Not connected to BLE device');
        return false;
    }
    
    const lat = parseFloat(document.getElementById(`poi${poiNum}Lat`).value);
    const lon = parseFloat(document.getElementById(`poi${poiNum}Lon`).value);
    const enabled = document.getElementById(`poi${poiNum}Enabled`).checked ? 1 : 0;
    
    if ((isNaN(lat) || isNaN(lon)) && enabled) {
        alert(`POI ${poiNum} has invalid coordinates but is enabled. Please correct.`);
        return false;
    }
    
    const command = `POI:${poiNum}:${lat}:${lon}:${enabled}`;
    const result = await sendCommand(command);
    
    if (result) {
        console.log(`POI ${poiNum} updated successfully`);
        return true;
    } else {
        console.error(`Failed to update POI ${poiNum}`);
        return false;
    }
}

// Update all POIs to the device - modified to use the new updateSinglePOI function
async function updateAllPOIs() {
    if (!rxCharacteristic) {
        alert('Please connect to the device first');
        return;
    }
    
    let success = true;
    
    for (let i = 1; i <= 3; i++) {
        const result = await updateSinglePOI(i);
        if (!result) {
            success = false;
        }
    }
    
    if (success) {
        alert('All POIs updated successfully!');
        // Reset "Set New" checkboxes
        for (let i = 1; i <= 3; i++) {
            document.getElementById(`poi${i}SetNew`).checked = false;
        }
        activeSetPOIIndex = -1;
    } else {
        alert('Some POIs failed to update. Please try again.');
    }
}

// Update fuel settings
async function updateFuel() {
    if (!rxCharacteristic) {
        alert('Please connect to the device first');
        return;
    }
    
    const fuelLevel = parseFloat(document.getElementById('newFuelLevel').value);
    const burnRate = parseFloat(document.getElementById('newBurnRate').value);
    
    if (isNaN(fuelLevel) || fuelLevel < 0) {
        alert('Please enter a valid fuel level');
        return;
    }
    
    if (isNaN(burnRate) || burnRate < 0) {
        alert('Please enter a valid burn rate');
        return;
    }
    
    const command = `FUEL:${fuelLevel}:${burnRate}`;
    const result = await sendCommand(command);
    
    if (result) {
        alert('Fuel settings updated successfully!');
        // Update current values with new values
        document.getElementById('currentFuelLevel').textContent = fuelLevel.toFixed(1);
        document.getElementById('currentBurnRate').textContent = burnRate.toFixed(1);
    } else {
        alert('Failed to update fuel settings. Please try again.');
    }
}

// Set operation mode
async function setMode(mode) {
    if (!rxCharacteristic) {
        alert('Please connect to the device first');
        return;
    }
    
    const command = `MODE:${mode}`;
    const result = await sendCommand(command);
    
    if (result) {
        updateModeButtons(mode);
    } else {
        alert('Failed to change mode. Please try again.');
    }
}

// Update mode button UI
function updateModeButtons(mode) {
    if (mode === 1) { // Flying mode
        document.getElementById('flyingModeBtn').classList.add('active');
        document.getElementById('walkingModeBtn').classList.remove('active');
    } else if (mode === 2) { // Walking mode
        document.getElementById('flyingModeBtn').classList.remove('active');
        document.getElementById('walkingModeBtn').classList.add('active');
    }
}

// Function to disable a POI
async function disablePOI(poiNum) {
    if (!rxCharacteristic) {
        alert('Please connect to the device first');
        return;
    }
    
    const lat = parseFloat(document.getElementById(`poi${poiNum}Lat`).value);
    const lon = parseFloat(document.getElementById(`poi${poiNum}Lon`).value);
    // Set enabled to 0 (disabled)
    const enabled = 0;
    
    const command = `POI:${poiNum}:${lat}:${lon}:${enabled}`;
    const result = await sendCommand(command);
    
    if (result) {
        // Update UI to reflect disabled state
        document.getElementById(`poi${poiNum}Enabled`).checked = false;
        updatePOIStatusIndicator(poiNum);
        hidePOIMarker(poiNum - 1);
        updateDisableButtonState(poiNum);
        alert(`POI ${poiNum} has been disabled.`);
    } else {
        alert('Failed to disable POI. Please try again.');
    }
}

// Update the disable button state based on the POI enabled state
function updateDisableButtonState(poiNum) {
    const enabled = document.getElementById(`poi${poiNum}Enabled`).checked;
    const disableBtn = document.getElementById(`poi${poiNum}DisableBtn`);
    
    if (enabled) {
        disableBtn.disabled = false;
    } else {
        disableBtn.disabled = true;
    }
}

// Check if browser supports Web Bluetooth
if (!navigator.bluetooth) {
    alert('Your browser does not support Web Bluetooth. Please use Chrome or Edge.');
    document.getElementById('connectButton').disabled = true;
}
