/**
 * Debugging utilities for ENAV web interface
 */

// Enable or disable verbose debugging
const DEBUG_ENABLED = true;

// Enhanced console logging
function debugLog(...args) {
    if (DEBUG_ENABLED) {
        console.log(`[${new Date().toISOString().slice(11, 23)}]`, ...args);
    }
}

// Monitor map events
function monitorMapEvents(map, mapName) {
    if (!DEBUG_ENABLED) return;
    
    const events = ['click', 'move', 'zoom', 'resize', 'load'];
    events.forEach(event => {
        map.on(event, (e) => {
            debugLog(`${mapName} event:`, event, e);
        });
    });
}

// Monitor DOM element changes
function monitorElementValue(elementId) {
    if (!DEBUG_ENABLED) return;
    
    const element = document.getElementById(elementId);
    if (!element) {
        debugLog(`Element not found: ${elementId}`);
        return;
    }
    
    // Use MutationObserver for select elements
    if (element.tagName === 'SELECT' || element.tagName === 'INPUT' || element.tagName === 'TEXTAREA') {
        element.addEventListener('change', (e) => {
            debugLog(`Element ${elementId} changed:`, element.value);
        });
    }
}

// Helper to check if coordinates are valid
function isValidCoordinate(lat, lon) {
    return !isNaN(lat) && !isNaN(lon) && 
           lat >= -90 && lat <= 90 && 
           lon >= -180 && lon <= 180;
}

// Check all POI fields
function debugCheckPOIFields() {
    for (let i = 1; i <= 3; i++) {
        const latField = document.getElementById(`poi${i}Lat`);
        const lonField = document.getElementById(`poi${i}Lon`);
        const enabledCheckbox = document.getElementById(`poi${i}Enabled`);
        
        if (latField && lonField && enabledCheckbox) {
            debugLog(`POI ${i}:`, {
                lat: latField.value,
                lon: lonField.value,
                enabled: enabledCheckbox.checked,
                validCoords: isValidCoordinate(parseFloat(latField.value), parseFloat(lonField.value))
            });
        } else {
            debugLog(`POI ${i}: Missing fields`);
        }
    }
}

// Initialize debugging
function initDebugging() {
    if (DEBUG_ENABLED) {
        debugLog('Debug mode enabled');
        
        // Monitor POI fields
        for (let i = 1; i <= 3; i++) {
            monitorElementValue(`poi${i}Lat`);
            monitorElementValue(`poi${i}Lon`);
            monitorElementValue(`poi${i}Enabled`);
            monitorElementValue(`poi${i}MapCoords`);
        }
        
        // Add a global debugging command
        window.debugEnav = {
            checkPOIs: debugCheckPOIFields,
            isValidCoord: isValidCoordinate,
            log: debugLog
        };
    }
}

// Run on DOM load
document.addEventListener('DOMContentLoaded', initDebugging);

// Expose globally
window.debugLog = debugLog;
window.monitorMapEvents = monitorMapEvents;
