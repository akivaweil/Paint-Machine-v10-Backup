#ifndef HTML_CONTENT_H
#define HTML_CONTENT_H
#include <Arduino.h>

const char HTML_PROGMEM[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">

<head>
    <title>ESP32 Stepper Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <style>
        :root {
            --primary: #4CAF50;
            --primary-light: #81C784;
            --primary-dark: #388E3C;
            --primary-hover: #5cb860;
            --secondary: #a142f4;
            --secondary-light: #b46ff6;
            --secondary-dark: #9138e0;
            --blue: #42a5f5;
            --blue-light: #64b5f6;
            --blue-dark: #1e88e5;
            --yellow: #fbc02d;
            --yellow-light: #fdd835;
            --yellow-dark: #f9a825;
            --background: #121418;
            --card-bg: #1e1f24;
            --text: #f8f9fa;
            --text-secondary: #b0b8c4;
            --text-muted: #9ca3af;
            --border: #333;
            --divider: #2d2e35;
            --input-bg: #2c2d32;
            --shadow-sm: 0 2px 4px rgba(0, 0, 0, .25);
            --shadow-md: 0 8px 24px rgba(0, 0, 0, .25);
            --shadow-lg: 0 12px 32px rgba(0, 0, 0, .3);
            --radius-sm: 6px;
            --radius-md: 10px;
            --radius-lg: 14px;
            --transition-fast: 0.15s ease;
            --transition-normal: 0.25s ease;
        }
        
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }
        
        body {
            background: var(--background);
            color: var(--text);
            font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
            margin: 0;
            padding: 16px;
            display: flex;
            flex-direction: column;
            align-items: center;
            font-size: 16px;
            line-height: 1.5;
            min-height: 100vh;
        }
        
        .container {
            width: 100%;
            max-width: 1100px;
            margin: 0 auto;
        }
        
        h1, h2, h3, h4 {
            color: var(--text);
            font-weight: 600;
            line-height: 1.2;
        }
        
        h1 {
            font-size: 1.75rem;
            margin-bottom: 1.5rem;
        }
        
        h2 {
            font-size: 1.25rem;
            margin-bottom: 1rem;
        }
        
        h3 {
            font-size: 1rem;
            margin-bottom: 0.5rem;
            color: var(--text-muted);
        }
        
        /* Status Display */
        #machineStatusDisplay {
            background: rgba(255, 255, 255, 0.06);
            border: 1px solid var(--border);
            border-radius: var(--radius-md);
            padding: 12px 20px;
            margin: 0 auto 24px auto;
            font-size: 1.1rem;
            font-weight: 500;
            color: var(--text-secondary);
            text-align: center;
            min-height: 25px;
            width: 100%;
            max-width: 800px;
            transition: background-color var(--transition-normal);
        }
        
        /* Connection Status */
        #connectionStatus {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 5px 10px;
            border-radius: var(--radius-sm);
            font-size: 12px;
            font-weight: 600;
            z-index: 1000;
            transition: opacity var(--transition-normal), background-color var(--transition-normal);
        }
        
        /* Main Card Styles */
        .main-card {
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: var(--radius-lg);
            padding: 24px;
            box-shadow: var(--shadow-md);
            transition: transform var(--transition-normal), box-shadow var(--transition-normal);
        }
        
        .main-card:hover {
            transform: translateY(-2px);
            box-shadow: var(--shadow-lg);
        }
        
        /* Top Controls Container */
        .top-controls-container {
            display: flex;
            justify-content: center;
            align-items: flex-start;
            flex-wrap: wrap;
            gap: 24px;
            margin-bottom: 32px;
            width: 100%;
            max-width: 1100px;
        }
        
        /* Integrated Main Card */
        .integrated-main-card {
            flex: 1 1 520px;
            border: 1px solid rgba(76, 175, 80, 0.2);
            padding: 28px;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            gap: 20px;
            min-width: 300px;
            max-width: 600px;
        }
        
        /* Pressure Controls */
        #pressureControlGroup {
            flex: 0 0 auto;
            padding: 24px;
            width: 260px;
            align-self: stretch;
            border: 1px solid rgba(161, 66, 244, 0.2);
        }
        
        #pressureControlGroup h2 {
            color: var(--secondary);
            font-size: 1.25rem;
            margin: 0 0 20px 0;
            text-align: center;
        }
        
        /* Divider */
        .main-divider {
            width: 100%;
            height: 1px;
            background: linear-gradient(90deg, var(--divider) 0%, var(--border) 100%);
            margin: 16px 0;
            border: none;
        }
        
        /* Button Styles */
        .main-btn {
            position: relative;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
            padding: 12px 20px;
            border-radius: var(--radius-md);
            background: linear-gradient(90deg, var(--primary) 60%, var(--primary-dark) 100%);
            color: white;
            font-size: 1rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.04em;
            cursor: pointer;
            box-shadow: var(--shadow-sm), inset 0 1px 0 rgba(255, 255, 255, 0.1);
            transition: background var(--transition-normal), transform var(--transition-fast), box-shadow var(--transition-normal);
            min-width: 130px;
            border: none;
            outline: none;
        }
        
        .main-btn .btn-icon {
            font-size: 1.25em;
            display: inline-block;
        }
        
        .main-btn:hover,
        .main-btn:focus {
            background: linear-gradient(90deg, var(--primary-hover) 60%, var(--primary) 100%);
            transform: translateY(-2px);
            box-shadow: var(--shadow-md), inset 0 1px 0 rgba(255, 255, 255, 0.15);
        }
        
        .main-btn:active {
            transform: scale(0.98);
            transition-duration: 0.05s;
        }
        
        .main-btn:disabled {
            background: #52525b;
            cursor: not-allowed;
            box-shadow: none;
            transform: none;
            opacity: 0.7;
        }
        
        /* Icon Styling */
        .btn-icon {
            display: flex;
            align-items: center;
            justify-content: center;
            transition: transform 0.2s ease;
        }

        .main-btn:hover .btn-icon {
            transform: scale(1.1);
        }

        .main-btn:active .btn-icon {
            transform: scale(0.95);
        }

        /* Direction button specific styling */
        .direction-buttons-grid .btn-icon svg {
            stroke-width: 2.5;
            stroke-linecap: round;
            stroke-linejoin: round;
        }

        /* Utility button specific styling */
        .utility-buttons-grid .btn-icon svg {
            stroke-width: 2;
            fill: none;
        }
        
        .main-btn.highlight {
            background: linear-gradient(90deg, var(--yellow) 60%, var(--yellow-dark) 100%);
            color: #222;
        }
        
        .main-btn.highlight:hover,
        .main-btn.highlight:focus {
            background: linear-gradient(90deg, var(--yellow-light) 60%, var(--yellow) 100%);
            color: #111;
        }
        
        .main-btn.blue {
            background: linear-gradient(90deg, var(--blue) 60%, var(--blue-dark) 100%);
        }
        
        .main-btn.blue:hover,
        .main-btn.blue:focus {
            background: linear-gradient(90deg, var(--blue-light) 60%, var(--blue) 100%);
        }
        
        .main-btn.mode {
            background: linear-gradient(90deg, var(--secondary) 60%, var(--secondary-dark) 100%);
        }
        
        .main-btn.mode:hover,
        .main-btn.mode:focus {
            background: linear-gradient(90deg, var(--secondary-light) 60%, var(--secondary) 100%);
        }
        
        /* Direction Buttons Grid */
        .direction-buttons-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            grid-template-rows: repeat(3, auto);
            gap: 12px;
            width: 100%;
            place-items: center;
        }
        
        /* Utility Buttons Grid */
        .utility-buttons-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 12px;
            width: 100%;
            place-items: center;
        }
        
        .utility-buttons-grid .main-btn {
            height: 64px;
            width: 100%;
            padding: 14px 20px;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        
        /* Toggle Switch */
        .toggle-container {
            width: 100%;
            padding: 16px;
            margin-bottom: 16px;
            border-radius: var(--radius-md);
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid rgba(161, 66, 244, 0.2);
            display: flex;
            flex-direction: column;
            align-items: center;
            transition: background-color var(--transition-normal), border-color var(--transition-normal);
        }
        
        .toggle-container:hover {
            background: rgba(255, 255, 255, 0.05);
            border-color: rgba(161, 66, 244, 0.3);
        }
        
        .toggle-container:last-child {
            margin-bottom: 0;
        }
        
        .toggle-container h3 {
            color: var(--text-muted);
            font-size: 1rem;
            margin: 0 0 12px 0;
            text-align: center;
        }
        
        .toggle-switch {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 12px;
            cursor: pointer;
            user-select: none;
            margin: 0 auto;
            padding: 5px 0 8px 0;
        }
        
        .toggle-switch input[type="checkbox"] {
            display: none;
        }
        
        .toggle-switch .slider {
            position: relative;
            width: 60px;
            height: 34px;
            background: #3a3b41;
            border-radius: 17px;
            transition: all var(--transition-normal);
            box-shadow: inset 0 2px 8px rgba(0, 0, 0, 0.15);
        }
        
        .toggle-switch .slider:before {
            content: "";
            position: absolute;
            left: 4px;
            top: 4px;
            width: 26px;
            height: 26px;
            background: white;
            border-radius: 50%;
            transition: transform var(--transition-normal);
            box-shadow: 0 2px 6px rgba(0, 0, 0, 0.2);
        }
        
        .toggle-switch input:checked+.slider {
            background: linear-gradient(90deg, var(--secondary) 60%, var(--secondary-dark) 100%);
        }
        
        .toggle-switch input:checked+.slider:before {
            transform: translateX(26px);
        }
        
        .toggle-label {
            min-width: 40px;
            font-weight: 600;
            color: var(--text-muted);
            letter-spacing: 0.04em;
            font-size: 1.1rem;
            transition: color var(--transition-normal);
            text-align: center;
        }
        
        .toggle-switch input:checked~.toggle-label {
            color: var(--secondary);
        }
        
        /* Pattern Settings */
        .pattern-settings-container {
            max-width: 900px;
            margin: 0 auto 32px auto;
            width: 100%;
        }
        
        .pattern-settings-header {
            color: var(--primary);
            font-size: 1.25rem;
            cursor: pointer;
            user-select: none;
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 16px 20px;
            background: rgba(255, 255, 255, 0.04);
            border-radius: var(--radius-md);
            border: 1px solid rgba(76, 175, 80, 0.2);
            transition: background-color var(--transition-normal), border-color var(--transition-normal);
            margin-bottom: 0;
        }
        
        .pattern-settings-header:hover {
            background: rgba(255, 255, 255, 0.08);
            border-color: rgba(76, 175, 80, 0.3);
        }
        
        .pattern-settings-header .toggle-indicator {
            font-size: 0.8em;
            transition: transform var(--transition-normal);
        }
        
        .pattern-settings-content-wrapper {
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: 0 0 var(--radius-md) var(--radius-md);
            border-top: none;
            padding: 24px;
            box-shadow: var(--shadow-md);
            overflow: hidden;
            transition: max-height var(--transition-normal), padding var(--transition-normal), opacity var(--transition-normal);
            max-height: 2000px;
            opacity: 1;
        }
        
        .pattern-settings-content-wrapper.collapsed {
            max-height: 0;
            padding-top: 0;
            padding-bottom: 0;
            opacity: 0;
            box-shadow: none;
        }
        
        /* Tab Navigation */
        .pattern-tabs {
            display: flex;
            flex-wrap: wrap;
            justify-content: center;
            gap: 12px;
            margin-bottom: 20px;
        }
        
        .pattern-tab {
            background-color: rgba(255, 255, 255, 0.08);
            color: var(--text);
            border: none;
            padding: 12px 20px;
            font-size: 1rem;
            border-radius: var(--radius-md);
            cursor: pointer;
            transition: background-color var(--transition-normal), transform var(--transition-normal), box-shadow var(--transition-normal);
        }
        
        .pattern-tab:hover {
            background-color: rgba(255, 255, 255, 0.12);
        }
        
        .pattern-tab.active {
            background-color: var(--primary);
            transform: translateY(-3px);
            box-shadow: var(--shadow-md);
        }
        
        /* Tab Content */
        .pattern-tab-content {
            display: none;
            width: 100%;
        }
        
        .pattern-tab-content.active {
            display: block;
            animation: fadeIn 0.3s ease;
        }
        
        @keyframes fadeIn {
            from { opacity: 0; }
            to { opacity: 1; }
        }
        
        /* Pattern Settings Grid */
        .pattern-settings-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 16px;
            margin-bottom: 20px;
        }
        
        .pattern-setting-group {
            background: rgba(255, 255, 255, 0.04);
            border-radius: var(--radius-md);
            padding: 16px;
            display: flex;
            flex-direction: column;
            align-items: center;
            transition: background-color var(--transition-normal);
        }
        
        .pattern-setting-group:hover {
            background: rgba(255, 255, 255, 0.06);
        }
        
        .pattern-setting-group h3 {
            color: var(--text-muted);
            font-size: 0.95rem;
            margin-top: 0;
            margin-bottom: 12px;
        }
        
        .setting-input {
            background: var(--input-bg);
            color: var(--text);
            border: 1px solid #4a4a50;
            border-radius: var(--radius-sm);
            padding: 8px 12px;
            font-size: 0.95rem;
            width: 90px;
            text-align: center;
            transition: border-color var(--transition-normal), box-shadow var(--transition-normal);
        }
        
        .setting-input:focus {
            outline: none;
            border-color: var(--primary);
            box-shadow: 0 0 0 2px rgba(76, 175, 80, 0.2);
        }
        
        /* Hide number input spinner arrows */
        input[type="number"]::-webkit-inner-spin-button,
        input[type="number"]::-webkit-outer-spin-button {
            -webkit-appearance: none;
            margin: 0;
        }
        
        input[type="number"] {
            -moz-appearance: textfield;
        }
        
        .labeled-inputs {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
            flex-wrap: wrap;
        }
        
        .labeled-inputs label {
            margin-right: 4px;
            font-size: 0.9rem;
            color: var(--text-secondary);
        }
        
        .labeled-inputs .setting-input {
            width: 70px;
        }
        
        .setting-inputs {
            display: flex;
            gap: 12px;
            width: 100%;
            justify-content: center;
        }
        
        .settings-controls {
            display: flex;
            justify-content: center;
            gap: 32px;
            margin-top: 20px;
        }
        
        .settings-controls .main-btn {
            min-width: 160px;
            padding: 12px 24px;
            font-size: 1rem;
        }
        
        /* Responsive Design */
        @media (max-width: 768px) {
            body {
                padding: 12px;
                font-size: 15px;
            }
            
            .pattern-settings-container {
                max-width: 100%;
            }
            
            .pattern-settings-grid {
                grid-template-columns: repeat(2, 1fr);
                gap: 12px;
            }
            
            .pattern-setting-group {
                padding: 12px;
            }
            
            .main-btn {
                min-width: 110px;
                font-size: 0.9rem;
                padding: 10px 16px;
            }
            
            .utility-buttons-grid .main-btn {
                height: 56px;
            }
            
            .integrated-main-card {
                padding: 20px;
            }
            
            #pressureControlGroup {
                padding: 20px;
                width: 240px;
            }
        }
        
        @media (max-width: 480px) {
            body {
                padding: 8px;
                font-size: 14px;
            }
            
            .pattern-settings-grid {
                grid-template-columns: 1fr;
            }
            
            .pattern-tabs {
                flex-direction: column;
                align-items: stretch;
            }
            
            .pattern-setting-group {
                padding: 12px 8px;
            }
            
            .setting-input {
                width: 80px;
            }
            
            .main-card {
                padding: 16px;
            }
            
            .integrated-main-card {
                padding: 16px;
                min-width: 0;
            }
            
            #pressureControlGroup {
                width: 100%;
                margin-top: 16px;
            }
            
            .top-controls-container {
                flex-direction: column;
                align-items: center;
            }
            
            .direction-buttons-grid,
            .utility-buttons-grid {
                gap: 8px;
            }
            
            .main-btn {
                min-width: 90px;
                font-size: 0.85rem;
                padding: 8px 12px;
            }
            
            .utility-buttons-grid .main-btn {
                height: 48px;
            }
        }

        /* Multiple Coats Section Styling */
        .multiple-coats-container {
            width: 100%;
            margin-top: 24px; /* Space above this section */
            padding: 20px;
            background: rgba(255, 255, 255, 0.04); /* Subtle background to group elements */
            border: 1px solid var(--divider);
            border-radius: var(--radius-md);
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 18px; /* Consistent gap between title, controls, and button */
        }

        .multiple-coats-container h3 {
            /* Uses general h3 styles for font-size, color, weight */
            margin-bottom: 0; /* Remove default margin as gap is handled by flex */
            text-align: center;
        }

        .multiple-coats-controls { /* Groups the two input fields */
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); /* Responsive: side-by-side or stacked */
            gap: 16px; /* Gap between the two input groups */
            width: 100%;
            max-width: 450px; /* Max width for the input controls area */
        }

        .coats-input-group {
            display: flex; /* Label above input */
            flex-direction: column;
            align-items: flex-start; /* Align label to the left */
            gap: 6px; /* Space between label and input */
        }
        
        .coats-input-group label {
            font-size: 0.9rem; /* Slightly smaller label text */
            color: var(--text-muted);
            margin-left: 2px; /* Small indent for the label */
        }

        .coats-input-group .setting-input {
            width: 100%; /* Input takes full width of its container */
            padding: 10px 12px; /* Custom padding for these inputs */
            /* Inherits text-align: center from general .setting-input */
        }

        #paintMultipleCoatsBtn {
            grid-column: 1 / -1; /* This makes the button span all columns of its parent grid */
            /* width and max-width are removed as the grid layout handles this */
            padding-top: 14px;
            padding-bottom: 14px;
        }
    </style>
    <script>
        // Global variables
        var websocket;
        var isWebSocketConnected = false;
        var reconnectInterval;
        var connectionAttempts = 0;
        var connectionStatus = document.createElement('div');
        
        // Initialize WebSocket connection
        function initWebSocket() {
            // Create status indicator if it doesn't exist
            if (!document.getElementById('connectionStatus')) {
                connectionStatus.id = 'connectionStatus';
                connectionStatus.style.position = 'fixed';
                connectionStatus.style.top = '10px';
                connectionStatus.style.right = '10px';
                connectionStatus.style.padding = '5px 10px';
                connectionStatus.style.borderRadius = '5px';
                connectionStatus.style.fontSize = '12px';
                connectionStatus.style.fontWeight = 'bold';
                connectionStatus.style.zIndex = '1000';
                connectionStatus.style.backgroundColor = '#ff5555';
                connectionStatus.style.color = 'white';
                connectionStatus.textContent = 'Connecting...';
                document.body.appendChild(connectionStatus);
            }
            
            // Update status
            updateConnectionStatus('connecting');
            
            // Use dynamic hostname instead of hardcoded IP
            var wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            var wsHost = window.location.hostname + ':81';
            var gateway = wsProtocol + '//' + wsHost + '/';
            
            console.log('Trying to open a WebSocket connection to: ' + gateway);
            
            try {
                websocket = new WebSocket(gateway);
                
                websocket.onopen = function(event) {
                    console.log('Connection opened');
                    isWebSocketConnected = true;
                    connectionAttempts = 0;
                    clearInterval(reconnectInterval);
                    updateConnectionStatus('connected');
                    
                    // Request current status after connection is established
                    setTimeout(function() {
                        sendCommand('GET_STATUS');
                        
                        // If we're initializing pattern settings, load them now
                        if (window.needToLoadPatternSettings) {
                            window.needToLoadPatternSettings = false;
                            sendCommand('GET_PAINT_SETTINGS');
                        }
                        
                        // Request PNP settings
                        if (websocket.readyState === WebSocket.OPEN) {
                            websocket.send(JSON.stringify({ command: "GET_PNP_SETTINGS" }));
                            console.log("Requested PNP settings");
                        }
                    }, 500);
                };
                
                websocket.onclose = function(event) {
                    console.log('Connection closed');
                    isWebSocketConnected = false;
                    updateConnectionStatus('disconnected');
                    scheduleReconnect();
                };
                
                websocket.onmessage = function(event) {
                    console.log('Received message: ' + event.data);
                    handleWebSocketMessage(event);
                };
                
                websocket.onerror = function(event) {
                    console.error('WebSocket error: ', event);
                    isWebSocketConnected = false;
                    updateConnectionStatus('error');
                    scheduleReconnect();
                };
            } catch (e) {
                console.error('Exception creating WebSocket: ' + e.message);
                updateConnectionStatus('error', e.message);
                scheduleReconnect();
            }
        }
        
        // Update connection status indicator
        function updateConnectionStatus(status, message) {
            const connectionStatus = document.getElementById('connectionStatus');
            if (!connectionStatus) {
                console.error("Connection status element not found!");
                return;
            }
            switch(status) {
                case 'connected':
                    connectionStatus.style.backgroundColor = '#4CAF50';
                    connectionStatus.textContent = 'Connected';
                    connectionStatus.style.opacity = '0.8';
                    // Fade out after 3 seconds
                    setTimeout(function() {
                        connectionStatus.style.opacity = '0.3';
                    }, 3000);
                    break;
                case 'disconnected':
                    connectionStatus.style.backgroundColor = '#ff5555';
                    connectionStatus.textContent = 'Disconnected - Reconnecting...';
                    connectionStatus.style.opacity = '1';
                    break;
                case 'connecting':
                    connectionStatus.style.backgroundColor = '#FFA500';
                    connectionStatus.textContent = 'Connecting...';
                    connectionStatus.style.opacity = '1';
                    break;
                case 'error':
                    connectionStatus.style.backgroundColor = '#ff5555';
                    connectionStatus.textContent = 'Connection Error' + (message ? ': ' + message : '');
                    connectionStatus.style.opacity = '1';
                    break;
            }
        }
        
        // Schedule reconnection with tiered intervals and timeout
        function scheduleReconnect() {
            if (reconnectInterval) {
                clearInterval(reconnectInterval);
            }

            connectionAttempts++;
            var delay;

            // Tier 1: 0.25s interval for the first 60 seconds (240 attempts)
            if (connectionAttempts <= 240) {
                delay = 250;
            } 
            // Tier 2: 0.5s interval for the next 120 seconds (up to 3 min total, 480 attempts)
            else if (connectionAttempts <= 480) {
                delay = 500;
            } 
            // Stop trying after 3 minutes
            else {
                console.log("Stopped attempting to reconnect after 3 minutes.");
                updateConnectionStatus('error', 'Stopped reconnecting after 3 min timeout.'); // Update status
                return; // Do not schedule another attempt
            }

            console.log(`Scheduling reconnect in ${delay}ms (attempt ${connectionAttempts})`);

            reconnectInterval = setTimeout(function() {
                if (!isWebSocketConnected) {
                    initWebSocket();
                }
            }, delay);
        }
        
        // Handle incoming WebSocket messages
        function handleWebSocketMessage(message) {
            try {
                // Try to parse message as JSON
                const data = JSON.parse(message.data);
                
                // Handle PNP settings data
                if (data.event === "pnp_settings") {
                    console.log("Received PNP settings:", data);
                    if (data.pnp_x_speed !== undefined) document.getElementById('pnp_x_speed').value = data.pnp_x_speed;
                    if (data.pnp_x_accel !== undefined) document.getElementById('pnp_x_accel').value = data.pnp_x_accel;
                    if (data.pnp_y_speed !== undefined) document.getElementById('pnp_y_speed').value = data.pnp_y_speed;
                    if (data.pnp_y_accel !== undefined) document.getElementById('pnp_y_accel').value = data.pnp_y_accel;
                    return;
                }
                
                // Handle other JSON messages if needed
                console.log("Received JSON message:", data);
                
            } catch (error) {
                // Not JSON, handle as text message
                const messageText = message.data;
                console.log("Received text message:", messageText);
                
                // --- State Handling --- 
                if (messageText.startsWith('STATE:')) {
                    const stateName = messageText.substring(6); // Get text after "STATE:"
                    const statusDisplay = document.getElementById('machineStatusDisplay');
                    if (statusDisplay) {
                        statusDisplay.textContent = stateName; // Update status display
                        console.log('Updated machine status display to: ' + stateName);
                    }
                    updateButtonStates(stateName); // Enable/disable buttons based on state
                }
                
                // Check for pressure pot status updates
                else if (messageText.startsWith('PRESSURE_POT_STATUS:')) {
                    const status = messageText.split(':')[1];
                    console.log('Updating pressure pot status to: ' + status);
                    
                    // Update UI toggle without triggering a new command
                    const toggle = document.getElementById('pressurePotToggle');
                    const label = document.getElementById('pressurePotToggleLabel');
                    
                    if (status === 'ON') {
                        toggle.checked = true;
                        label.textContent = 'ON';
                    } else {
                        toggle.checked = false;
                        label.textContent = 'OFF';
                    }
                }
                
                // Check for paint gun status updates
                else if (messageText.startsWith('PAINT_GUN_STATUS:')) {
                    const status = messageText.split(':')[1];
                    console.log('Updating paint gun status to: ' + status);
                    
                    // Update UI toggle without triggering a new command
                    const toggle = document.getElementById('paintGunToggle');
                    const label = document.getElementById('paintGunToggleLabel');
                    
                    if (status === 'ON') {
                        toggle.checked = true;
                        label.textContent = 'ON';
                    } else {
                        toggle.checked = false;
                        label.textContent = 'OFF';
                    }
                }
                
                // Handle settings messages
                else if (messageText.startsWith('SETTING:')) {
                    const parts = messageText.split(':');
                    if (parts.length >= 3) {
                        const setting = parts[1];
                        const value = parts[2];
                        updateSettingField(setting, value);
                    }
                }
                
                // Handle status messages
                else if (messageText.startsWith('STATUS:')) {
                    console.log('Status message: ' + messageText.substring(7));
                    // Could add notification display here
                }
            }
        }
        
        // --- Function to Enable/Disable Buttons based on State --- 
        function updateButtonStates(stateName) {
            console.log("Updating button states for state: " + stateName);
            
            // Determine if machine is in IDLE state
            const isIdle = (stateName === 'IDLE');
            // const canPerformActions = (stateName === 'IDLE' || stateName === 'PNP'); // Keep for other buttons if needed

            // Get other button elements (paint, home, clean, pnp start)
            const paintSide1Btn = document.getElementById('paintSide1Btn');
            const paintSide2Btn = document.getElementById('paintSide2Btn');
            const paintSide3Btn = document.getElementById('paintSide3Btn');
            const paintSide4Btn = document.getElementById('paintSide4Btn');
            const paintAllSidesBtn = document.getElementById('paintAllSidesBtn');
            const paintMultipleCoatsBtn = document.getElementById('paintMultipleCoatsBtn'); // Added
            const homeBtn = document.getElementById('homeBtn');
            const cleanGunBtn = document.getElementById('cleanGunBtn');
            const pnpButton = document.getElementById('pnpButton'); // Assuming this is the start PnP button
            // const pressurePotToggle = document.getElementById('pressurePotToggle'); // Toggles are handled separately or always on
            // const paintGunToggle = document.getElementById('paintGunToggle');

            // General action buttons (Painting, Homing, Cleaning, PnP Start)
            // Typically enabled only when IDLE. Adjust if PNP also allows some of these.
            const canPerformGeneralActions = isIdle || (stateName === 'PNP'); // Or simply use isIdle if PNP mode blocks these too

            if (paintSide1Btn) paintSide1Btn.disabled = !isIdle; // Only from IDLE
            if (paintSide2Btn) paintSide2Btn.disabled = !isIdle; // Only from IDLE
            if (paintSide3Btn) paintSide3Btn.disabled = !isIdle; // Only from IDLE
            if (paintSide4Btn) paintSide4Btn.disabled = !isIdle; // Only from IDLE
            if (paintAllSidesBtn) paintAllSidesBtn.disabled = !isIdle; // Only from IDLE
            if (paintMultipleCoatsBtn) paintMultipleCoatsBtn.disabled = !isIdle; // Only from IDLE
            if (homeBtn) homeBtn.disabled = !isIdle && !(stateName === 'ERROR'); // Allow homing from IDLE or ERROR state
            if (cleanGunBtn) cleanGunBtn.disabled = !isIdle; // Only from IDLE
            if (pnpButton) pnpButton.disabled = !isIdle; // Only from IDLE to start PnP cycle

            // Manual Control Elements
            const manualXInput = document.getElementById('manualX');
            const manualYInput = document.getElementById('manualY');
            const manualZInput = document.getElementById('manualZ');
            const manualAngleInput = document.getElementById('manualAngle');
            const manualMoveBtn = document.getElementById('manualMoveToBtn');
            const manualRotateCWBtn = document.getElementById('manualRotateCwBtn');
            const manualRotateCCWBtn = document.getElementById('manualRotateCcwBtn');

            // Enable manual controls and inputs only if IDLE
            if (manualXInput) manualXInput.disabled = !isIdle;
            if (manualYInput) manualYInput.disabled = !isIdle;
            if (manualZInput) manualZInput.disabled = !isIdle;
            if (manualAngleInput) manualAngleInput.disabled = !isIdle;
            if (manualMoveBtn) manualMoveBtn.disabled = !isIdle;
            if (manualRotateCWBtn) manualRotateCWBtn.disabled = !isIdle;
            if (manualRotateCCWBtn) manualRotateCCWBtn.disabled = !isIdle;
            
            // The enterManualModeBtn and exitManualModeBtn elements will be removed from HTML
            // So, no need to get or disable them here anymore.
        }

        // Send commands to the ESP32
        function sendCommand(command) {
            if (!websocket || websocket.readyState !== WebSocket.OPEN) {
                const wsState = websocket ? websocket.readyState : 'null';
                console.error(`WebSocket not connected. Command '${command}' failed. WebSocket state: ${wsState}, Connection attempts: ${connectionAttempts}`);
                
                // Check if we are still within the allowed reconnection attempts
                if (connectionAttempts <= 480) { // 480 is the current max attempts ceiling from scheduleReconnect
                    updateConnectionStatus('error', 'Not Connected! Attempting to reconnect...');
                    console.log("Calling initWebSocket() due to command attempt while disconnected.");
                    initWebSocket(); // Attempt to re-establish connection
                } else {
                    updateConnectionStatus('error', 'Not Connected. Max retries reached. Please refresh.');
                    console.warn("Max reconnection attempts reached. Command '" + command + "' not sent. User needs to refresh.");
                }
                
                // Existing queuing logic for specific commands
                if (command === 'GET_PAINT_SETTINGS') {
                    window.needToLoadPatternSettings = true;
                    console.log("GET_PAINT_SETTINGS command queued due to disconnection.");
                } else if (command !== 'GET_STATUS') {  // Don't queue status requests
                    // Attempt to resend the command after a short delay, hoping for reconnection
                    setTimeout(function() {
                        if (isWebSocketConnected && websocket && websocket.readyState === WebSocket.OPEN) {
                            console.log(`Retrying command '${command}' after a short delay upon reconnection.`);
                            websocket.send(command);
                        } else {
                            console.log(`Still not connected after delay. Queued command '${command}' not resent.`);
                        }
                    }, 2000); // Wait 2 seconds and check again
                }
                return false; // Indicate command was not sent
            }
            
            try {
                websocket.send(command);
                console.log(`Command sent: ${command}`);
                return true;
            } catch (e) {
                console.error(`Error sending command '${command}': ${e.message}`);
                updateConnectionStatus('error', `Send Error: ${e.message}`);
                return false;
            }
        }
        
        // Toggle Pressure Pot
        function togglePressurePot(enabled) {
            // Send the command to the device
            const command = enabled ? 'PRESSURE_POT_ON' : 'PRESSURE_POT_OFF';
            sendCommand(command);
            
            // Update the UI explicitly
            const toggle = document.getElementById('pressurePotToggle');
            const label = document.getElementById('pressurePotToggleLabel');
            
            // Set the toggle state
            toggle.checked = enabled;
            
            // Update the label
            label.textContent = enabled ? 'ON' : 'OFF';
            
            // Debug toggle state
            console.log(`Toggle Pressure Pot: ${enabled ? 'ON' : 'OFF'}, checked=${toggle.checked}`);
        }
        
        // Toggle Paint Gun
        function togglePaintGun(enabled) {
            // Send the command to the device
            const command = enabled ? 'PAINT_GUN_ON' : 'PAINT_GUN_OFF';
            sendCommand(command);
            
            // Update the UI explicitly
            const toggle = document.getElementById('paintGunToggle');
            const label = document.getElementById('paintGunToggleLabel');
            
            // Set the toggle state
            toggle.checked = enabled;
            
            // Update the label
            label.textContent = enabled ? 'ON' : 'OFF';
            
            // Debug toggle state
            console.log(`Toggle Paint Gun: ${enabled ? 'ON' : 'OFF'}, checked=${toggle.checked}`);
        }

        // Handle Pattern Tab Navigation
        function openPatternTab(tabId) {
            // Hide all tab contents
            var tabContents = document.getElementsByClassName('pattern-tab-content');
            for (var i = 0; i < tabContents.length; i++) {
                tabContents[i].classList.remove('active');
            }
            
            // Deactivate all tab buttons
            var tabButtons = document.getElementsByClassName('pattern-tab');
            for (var i = 0; i < tabButtons.length; i++) {
                tabButtons[i].classList.remove('active');
            }
            
            // Show the selected tab content
            document.getElementById(tabId).classList.add('active');
            
            // Activate the clicked tab button
            document.getElementById(tabId + 'Btn').classList.add('active');
        }
        
        // Update a pattern setting and send to the ESP32
        function updatePatternSetting(setting, value) {
            console.log(`Updating ${setting} to ${value}`);
            sendCommand(`SET_${setting}:${value}`);

            // Add preview commands for Z height and Servo angle
            if (setting.endsWith('ZHEIGHT')) {
                sendCommand(`MOVE_Z_PREVIEW:${value}`);
            } else if (setting.startsWith('SERVO_ANGLE')) {
                sendCommand(`MOVE_SERVO_PREVIEW:${value}`);
            }
        }
        
        // Load all pattern settings from the ESP32
        function loadPatternSettings() {
            console.log("Loading pattern settings");
            sendCommand('GET_PAINT_SETTINGS');
        }
        
        // Save all pattern settings to NVS
        function savePatternSettings() {
            console.log("Saving pattern settings");
            sendCommand('SAVE_PAINT_SETTINGS');
        }
        
        // Handle incoming settings from the ESP32
        function updateSettingField(setting, value) {
            const field = document.getElementById(setting);
            if (field) {
                field.value = value;
                console.log(`Updated ${setting} field to ${value}`);
            }
        }
        
        // Initialize pattern settings section when page loads
        function initPatternSettings() {
            // Activate the first tab by default
            openPatternTab('side3Tab');
            
            // Make settings section collapsible
            const header = document.getElementById('patternSettingsHeader');
            const content = document.getElementById('patternSettingsContent');
            if(header && content) {
                 header.onclick = function() {
                    header.classList.toggle('collapsed');
                    content.classList.toggle('collapsed');
                    
                    // Toggle the indicator text
                    const indicator = header.querySelector('.toggle-indicator');
                    if (indicator) {
                        indicator.textContent = content.classList.contains('collapsed') ? '▼' : '▲';
                    }
                 };
            } else {
                console.error("Could not find pattern settings header or content for collapsible feature.");
            }

            // Flag that we need to load settings once connected
            window.needToLoadPatternSettings = true;
            
            // If already connected, load immediately
            if (isWebSocketConnected && websocket && websocket.readyState === WebSocket.OPEN) {
                console.log("Loading pattern settings");
                window.needToLoadPatternSettings = false;
                sendCommand('GET_PAINT_SETTINGS');
            }
        }

        function updatePnpSettings() {
            const pnpXSpeed = document.getElementById('pnp_x_speed').value;
            const pnpXAccel = document.getElementById('pnp_x_accel').value;
            const pnpYSpeed = document.getElementById('pnp_y_speed').value;
            const pnpYAccel = document.getElementById('pnp_y_accel').value;
            
            const payload = {
                command: "update_pnp_settings",
                pnp_x_speed: parseInt(pnpXSpeed),
                pnp_x_accel: parseInt(pnpXAccel),
                pnp_y_speed: parseInt(pnpYSpeed),
                pnp_y_accel: parseInt(pnpYAccel)
            };
            
            if (websocket.readyState === WebSocket.OPEN) {
                websocket.send(JSON.stringify(payload));
                console.log("PNP settings sent to ESP32");
            } else {
                console.error("WebSocket not connected");
            }
        }

        function initPnpSettings() {
            // Request current PNP settings from the ESP32
            if (websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send(JSON.stringify({ command: "GET_PNP_SETTINGS" }));
                console.log("Requested PNP settings");
            }
            
            // Make PNP settings section collapsible
            const header = document.getElementById('pnpSettingsHeader');
            const content = document.getElementById('pnpSettingsContent');
            if(header && content) {
                header.onclick = function() {
                    header.classList.toggle('collapsed');
                    content.classList.toggle('collapsed');
                    
                    // Toggle the indicator text
                    const indicator = header.querySelector('.toggle-indicator');
                    if (indicator) {
                        indicator.textContent = content.classList.contains('collapsed') ? '▼' : '▲';
                    }
                };
            }
        }

        function sendManualMoveTo() {
            const x = document.getElementById('manualX').value.trim();
            const y = document.getElementById('manualY').value.trim();
            const z = document.getElementById('manualZ').value.trim(); // Trim, can be empty
            const angle = document.getElementById('manualAngle').value.trim(); // Trim, can be empty
            
            if (x === "" || y === "") { // Only X and Y are mandatory
                alert("Please fill at least X and Y fields for manual move. Z and Angle are optional.");
                return;
            }
            // Construct command, allowing empty Z and Angle (backend handles this)
            const command = `MANUAL_MOVE_TO:${x},${y},${z},${angle}`;
            sendCommand(command);
        }

        function paintMultipleCoats() {
            const numCoatsInput = document.getElementById('numCoats');
            const interCoatDelayInput = document.getElementById('interCoatDelay'); // New

            if (!numCoatsInput || !interCoatDelayInput) { // Check both
                console.error("Could not find numCoats or interCoatDelay input element.");
                alert("Error: Input elements for coats/delay not found.");
                return;
            }
            const numCoats = numCoatsInput.value;
            const interCoatDelay = interCoatDelayInput.value; // New

            // Basic validation
            const numCoatsInt = parseInt(numCoats);
            const interCoatDelayInt = parseInt(interCoatDelay); // New

            if (isNaN(numCoatsInt) || numCoatsInt < 1 || numCoatsInt > 10) { 
                alert("Please enter a valid number of coats (1-10).");
                return;
            }
            if (isNaN(interCoatDelayInt) || interCoatDelayInt < 0 || interCoatDelayInt > 600) { // Max 600s (10 min)
                alert("Please enter a valid delay between coats (0-600 seconds).");
                return;
            }

            const command = `PAINT_MULTIPLE_COATS:${numCoatsInt}:${interCoatDelayInt}`; // Updated command
            sendCommand(command);
            console.log("Sending command: " + command);
        }

        // Initialize the app when the page loads
        window.addEventListener('load', function() {
            initWebSocket();
            
            // Add toggle indicators to collapsible sections
            const patternSettingsHeader = document.getElementById('patternSettingsHeader');
            if (patternSettingsHeader) {
                const indicator = document.createElement('span');
                indicator.className = 'toggle-indicator';
                indicator.textContent = '▲';
                patternSettingsHeader.appendChild(indicator);
            }
            
            const pnpSettingsHeader = document.getElementById('pnpSettingsHeader');
            if (pnpSettingsHeader) {
                const indicator = document.createElement('span');
                indicator.className = 'toggle-indicator';
                indicator.textContent = '▲';
                pnpSettingsHeader.appendChild(indicator);
            }
            
            const manualControlHeader = document.getElementById('manualControlHeader');
            if (manualControlHeader) {
                const indicator = document.createElement('span');
                indicator.className = 'toggle-indicator';
                indicator.textContent = '▲';
                manualControlHeader.appendChild(indicator);
            }
            
            initPatternSettings();
            initPnpSettings();
            
            // Initially load settings
            loadPatternSettings();
        });
    </script>
</head>

<body>
    <!-- Machine Status Display -->
    <div id="machineStatusDisplay">Machine status will appear here...</div>

    <!-- Main Controls Container -->
    <div class="top-controls-container">
        <!-- Integrated Main Control Card with all primary buttons -->
        <div class="integrated-main-card main-card">
            <!-- Paint Direction Buttons -->
            <div class="direction-buttons-grid">
                <!-- SIDE 1 button at top center -->
                <button id="paintSide1Btn" class="main-btn" title="Paint Side 1" aria-label="Paint Side 1" onclick="sendCommand('PAINT_SIDE_1')" style="grid-column: 2; grid-row: 1;">
                    <span class="btn-icon" aria-hidden="true">
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                            <path d="M12 4V20M12 20L6 14M12 20L18 14" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                        </svg>
                    </span>
                    <span class="btn-label">SIDE 1</span>
                </button>

                <!-- SIDE 4 button on left side -->
                <button id="paintSide4Btn" class="main-btn" title="Paint Side 4" aria-label="Paint Side 4" onclick="sendCommand('PAINT_SIDE_4')" style="grid-column: 1; grid-row: 2;">
                    <span class="btn-icon" aria-hidden="true">
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                            <path d="M4 12H20M20 12L14 6M20 12L14 18" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                        </svg>
                    </span>
                    <span class="btn-label">SIDE 4</span>
                </button>

                <!-- ALL SIDES button in center -->
                <button id="paintAllSidesBtn" class="main-btn highlight" title="Paint All Sides Once" aria-label="Paint All Sides Once" onclick="sendCommand('PAINT_ALL_SIDES')" style="grid-column: 2; grid-row: 2;">
                    <span class="btn-icon" aria-hidden="true">
                        <svg width="28" height="28" viewBox="0 0 28 28" fill="none" xmlns="http://www.w3.org/2000/svg" transform="rotate(90 14 14)">
                            <rect x="9" y="9" width="10" height="10" rx="1" stroke="currentColor" stroke-width="2.5"/>
                            <path d="M14 3V9" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M14 19V25" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M3 14H9" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M19 14H25" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M6 6L9 9" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M19 9L22 6" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M6 22L9 19" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M19 19L22 22" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                        </svg>
                    </span>
                    <span class="btn-label">PAINT ALL SIDES ONCE</span>
                </button>

                <!-- SIDE 2 button on right side -->
                <button id="paintSide2Btn" class="main-btn" title="Paint Side 2" aria-label="Paint Side 2" onclick="sendCommand('PAINT_SIDE_2')" style="grid-column: 3; grid-row: 2;">
                    <span class="btn-icon" aria-hidden="true">
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                            <path d="M20 12H4M4 12L10 6M4 12L10 18" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                        </svg>
                    </span>
                    <span class="btn-label">SIDE 2</span>
                </button>

                <!-- SIDE 3 button at bottom center -->
                <button id="paintSide3Btn" class="main-btn" title="Paint Side 3" aria-label="Paint Side 3" onclick="sendCommand('PAINT_SIDE_3')" style="grid-column: 2; grid-row: 3;">
                    <span class="btn-icon" aria-hidden="true">
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                            <path d="M12 20V4M12 4L6 10M12 4L18 10" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
                        </svg>
                    </span>
                    <span class="btn-label">SIDE 3</span>
                </button>
            </div>

            <!-- Multiple Coats Section -->
            <div class="multiple-coats-container">
                <h3>Multiple Coats</h3>
                <div class="multiple-coats-controls">
                    <div class="coats-input-group">
                        <label for="numCoats">Number of Coats:</label>
                        <input type="number" id="numCoats" class="setting-input" min="1" max="10" value="3" step="1">
                    </div>
                    <div class="coats-input-group">
                        <label for="interCoatDelay">Delay (s):</label>
                        <input type="number" id="interCoatDelay" class="setting-input" min="0" max="600" value="10" step="1">
                    </div>
                    <button id="paintMultipleCoatsBtn" class="main-btn highlight" title="Paint All Sides with Multiple Coats" aria-label="Paint All Sides with Multiple Coats" onclick="paintMultipleCoats()">
                        <span class="btn-icon" aria-hidden="true">
                            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                                <path d="M7 18H17V6H7V18Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                                <path d="M4 10V14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                                <path d="M20 10V14" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                                <path d="M7 10H17" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                                <path d="M7 14H17" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                            </svg>
                        </span>
                        <span class="btn-label">PAINT MULTIPLE COATS</span>
                    </button>
                </div> <!-- multiple-coats-controls div now closes AFTER the button -->
            </div>

            <div class="main-divider"></div>

            <!-- Utility Buttons -->
            <div class="utility-buttons-grid">
                <button id="homeBtn" class="main-btn blue" title="Home All Axes" aria-label="Home" onclick="sendCommand('HOME')">
                    <span class="btn-icon" aria-hidden="true">
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                            <path d="M5 12H3L12 3L21 12H19" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M5 12V19C5 19.5523 5.44772 20 6 20H18C18.5523 20 19 19.5523 19 19V12" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M9 20V14C9 13.4477 9.44772 13 10 13H14C14.5523 13 15 13.4477 15 14V20" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                        </svg>
                    </span>
                    <span class="btn-label">HOME</span>
                </button>
                <button id="cleanGunBtn" class="main-btn blue" title="Clean Paint Gun" aria-label="Clean Gun" onclick="sendCommand('CLEAN_GUN')">
                    <span class="btn-icon" aria-hidden="true">
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                            <path d="M14.5 8.5L16 7M18 5L16 7M16 7L19 10M19 10L21 12M19 10L17.5 11.5" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M3 21L10 14M6 18L8 20" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                            <path d="M13 7L8.5 11.5C7.83333 12.1667 6.9 13.5 7.5 14.5C8.1 15.5 9.5 14.5 10.5 13.5L15 9C15.6667 8.33333 16.9 7 16.5 6C16.1 5 14.5 5.5 13 7Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                        </svg>
                    </span>
                    <span class="btn-label">CLEAN GUN</span>
                </button>
                <button id="pnpButton" class="main-btn mode" title="Switch to Pick and Place Mode" aria-label="Pick and Place Mode" onclick="sendCommand('ENTER_PICKPLACE')">
                    <span class="btn-icon" aria-hidden="true">
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                            <path d="M7 14.5L5 14.5C3.89543 14.5 3 13.6046 3 12.5L3 11.5C3 10.3954 3.89543 9.5 5 9.5L19 9.5C20.1046 9.5 21 10.3954 21 11.5L21 12.5C21 13.6046 20.1046 14.5 19 14.5L17 14.5" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                            <rect x="7" y="11.5" width="10" height="8" rx="1" stroke="currentColor" stroke-width="2"/>
                            <path d="M9 11.5V7.5C9 6.39543 9.89543 5.5 11 5.5H13C14.1046 5.5 15 6.39543 15 7.5V11.5" stroke="currentColor" stroke-width="2"/>
                            <circle cx="10" cy="15.5" r="1" fill="currentColor"/>
                            <circle cx="14" cy="15.5" r="1" fill="currentColor"/>
                        </svg>
                    </span>
                    <span class="btn-label">PICK & PLACE</span>
                </button>
            </div>
        </div>

        <!-- Pressure Control Group -->
        <div class="main-card" id="pressureControlGroup">
            <h2>Pressure/Paint</h2>
            <div class="toggle-container">
                <h3>Pressure Pot</h3>
                <label class="toggle-switch">
                    <input type="checkbox" id="pressurePotToggle" onchange="togglePressurePot(this.checked)">
                    <span class="slider"></span>
                    <span class="toggle-label" id="pressurePotToggleLabel">OFF</span>
                </label>
            </div>

            <div class="toggle-container">
                <h3>Paint Gun</h3>
                <label class="toggle-switch">
                    <input type="checkbox" id="paintGunToggle" onchange="togglePaintGun(this.checked)">
                    <span class="slider"></span>
                    <span class="toggle-label" id="paintGunToggleLabel">OFF</span>
                </label>
            </div>
        </div>
    </div>
    
    <!-- Manual Control Section Placeholder -->
    <div class="pattern-settings-container" id="manualControlSection">
        <h2 id="manualControlHeader" class="pattern-settings-header">
            Manual Jog & Rotation
        </h2>
        <div class="pattern-settings-content-wrapper" id="manualControlContent">
            <div class="pattern-settings-grid">
                 <div class="pattern-setting-group">
                    <h3>Target Position (Steps/Angle)</h3>
                    <div class="setting-inputs labeled-inputs">
                        <label for="manualX">X:</label>
                        <input type="number" id="manualX" class="setting-input" placeholder="0">
                        <label for="manualY">Y:</label>
                        <input type="number" id="manualY" class="setting-input" placeholder="0">
                        <label for="manualZ">Z:</label>
                        <input type="number" id="manualZ" class="setting-input" placeholder="0">
                        <label for="manualAngle">Angle:</label>
                        <input type="number" id="manualAngle" class="setting-input" placeholder="90">
                    </div>
                    <button id="manualMoveToBtn" class="main-btn" onclick="sendManualMoveTo()" style="margin-top: 10px;">Move to Position</button>
                </div>
                <div class="pattern-setting-group">
                    <h3>Manual Rotation</h3>
                    <div class="setting-inputs">
                        <button id="manualRotateCwBtn" class="main-btn" onclick="sendCommand('MANUAL_ROTATE_CW')">Rotate CW 90&deg;</button>
                        <button id="manualRotateCcwBtn" class="main-btn" onclick="sendCommand('MANUAL_ROTATE_CCW')">Rotate CCW 90&deg;</button>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Pattern Settings Section with Tabs -->
    <div class="pattern-settings-container">
        <!-- Clickable Header for Toggling -->
        <h2 id="patternSettingsHeader" class="pattern-settings-header">
            Pattern Settings
        </h2>

        <!-- Content Wrapper for Collapsing -->
        <div class="pattern-settings-content-wrapper" id="patternSettingsContent">
            <!-- Tab Navigation -->
            <div class="pattern-tabs">
                <button class="pattern-tab" id="side1TabBtn" onclick="openPatternTab('side1Tab')">Side 1 Settings</button>
                <button class="pattern-tab" id="side2TabBtn" onclick="openPatternTab('side2Tab')">Side 2 Settings</button>
                <button class="pattern-tab" id="side3TabBtn" onclick="openPatternTab('side3Tab')">Side 3 Settings</button>
                <button class="pattern-tab" id="side4TabBtn" onclick="openPatternTab('side4Tab')">Side 4 Settings</button>
            </div>
            
            <div class="main-divider"></div>
            
            <!-- Side 1 Settings Tab Content -->
            <div class="pattern-tab-content" id="side1Tab">
                <div class="pattern-settings-grid">
                    <div class="pattern-setting-group">
                        <h3>Side 1 Starting Position</h3>
                        <div class="setting-inputs labeled-inputs">
                            <label for="side1StartX">X:</label>
                            <input type="number" id="side1StartX" class="setting-input" min="0" max="37" placeholder="0.0" onchange="updatePatternSetting('SIDE1STARTX', this.value)">
                            <label for="side1StartY">Y:</label>
                            <input type="number" id="side1StartY" class="setting-input" min="0" max="37" placeholder="0.0" onchange="updatePatternSetting('SIDE1STARTY', this.value)">
                        </div>
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Shift Distance</h3>
                        <input type="number" id="side1ShiftX" class="setting-input" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE1SHIFTX', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Sweep Distance</h3>
                        <input type="number" id="side1SweepY" class="setting-input" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE1SWEEPY', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Shift Speed</h3>
                        <input type="number" id="side1PaintingXSpeed" class="setting-input" max="20000" step="1000" placeholder="20" onchange="updatePatternSetting('SIDE1PAINTINGXSPEED', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Sweep Speed</h3>
                        <input type="number" id="side1PaintingYSpeed" class="setting-input" max="20000" step="1000" placeholder="20" onchange="updatePatternSetting('SIDE1PAINTINGYSPEED', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Z Height</h3>
                        <input type="number" id="side1ZHeight" class="setting-input" min="-2.5" max="0" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE1ZHEIGHT', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Servo Angle</h3>
                        <input type="number" id="servoAngleSide1" class="setting-input" min="30" max="100" step="1" placeholder="0" onchange="updatePatternSetting('SERVO_ANGLE_SIDE1', this.value)">
                    </div>
                </div>
            </div>

            <!-- Side 2 Settings Tab Content -->
            <div class="pattern-tab-content" id="side2Tab">
                <div class="pattern-settings-grid">
                    <div class="pattern-setting-group">
                        <h3>Side 2 Starting Position</h3>
                        <div class="setting-inputs labeled-inputs">
                            <label for="side2StartX">X:</label>
                            <input type="number" id="side2StartX" class="setting-input" min="0" max="37" placeholder="0.0" onchange="updatePatternSetting('SIDE2STARTX', this.value)">
                            <label for="side2StartY">Y:</label>
                            <input type="number" id="side2StartY" class="setting-input" min="0" max="37" placeholder="0.0" onchange="updatePatternSetting('SIDE2STARTY', this.value)">
                        </div>
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Shift Distance</h3>
                        <input type="number" id="side2ShiftX" class="setting-input" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE2SHIFTX', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Sweep Distance</h3>
                        <input type="number" id="side2SweepY" class="setting-input" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE2SWEEPY', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Shift Speed</h3>
                        <input type="number" id="side2PaintingXSpeed" class="setting-input" max="20000" step="1000" placeholder="20" onchange="updatePatternSetting('SIDE2PAINTINGXSPEED', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Sweep Speed</h3>
                        <input type="number" id="side2PaintingYSpeed" class="setting-input" max="20000" step="1000" placeholder="20" onchange="updatePatternSetting('SIDE2PAINTINGYSPEED', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Z Height</h3>
                        <input type="number" id="side2ZHeight" class="setting-input" min="-2.5" max="0" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE2ZHEIGHT', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Servo Angle</h3>
                        <input type="number" id="servoAngleSide2" class="setting-input" min="30" max="100" step="1" placeholder="0" onchange="updatePatternSetting('SERVO_ANGLE_SIDE2', this.value)">
                    </div>
                </div>
            </div>

            <!-- Side 3 Settings Tab Content -->
            <div class="pattern-tab-content" id="side3Tab">
                <div class="pattern-settings-grid">
                    <div class="pattern-setting-group">
                        <h3>Side 3 Starting Position</h3>
                        <div class="setting-inputs labeled-inputs">
                            <label for="side3StartX">X:</label>
                            <input type="number" id="side3StartX" class="setting-input" min="0" max="37" placeholder="0.0" onchange="updatePatternSetting('SIDE3STARTX', this.value)">
                            <label for="side3StartY">Y:</label>
                            <input type="number" id="side3StartY" class="setting-input" min="0" max="37" placeholder="0.0" onchange="updatePatternSetting('SIDE3STARTY', this.value)">
                        </div>
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Shift Distance</h3>
                        <input type="number" id="side3ShiftX" class="setting-input" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE3SHIFTX', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Sweep Distance</h3>
                        <input type="number" id="side3SweepY" class="setting-input" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE3SWEEPY', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Shift Speed</h3>
                        <input type="number" id="side3PaintingXSpeed" class="setting-input" max="20000" step="1000" placeholder="20" onchange="updatePatternSetting('SIDE3PAINTINGXSPEED', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Sweep Speed</h3>
                        <input type="number" id="side3PaintingYSpeed" class="setting-input" max="20000" step="1000" placeholder="20" onchange="updatePatternSetting('SIDE3PAINTINGYSPEED', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Z Height</h3>
                        <input type="number" id="side3ZHeight" class="setting-input" min="-2.5" max="0" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE3ZHEIGHT', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Servo Angle</h3>
                        <input type="number" id="servoAngleSide3" class="setting-input" min="30" max="100" step="1" placeholder="0" onchange="updatePatternSetting('SERVO_ANGLE_SIDE3', this.value)">
                    </div>
                </div>
            </div>

            <!-- Side 4 Settings Tab Content -->
            <div class="pattern-tab-content" id="side4Tab">
                <div class="pattern-settings-grid">
                    <div class="pattern-setting-group">
                        <h3>Side 4 Starting Position</h3>
                        <div class="setting-inputs labeled-inputs">
                            <label for="side4StartX">X:</label>
                            <input type="number" id="side4StartX" class="setting-input" min="0" max="37" placeholder="0.0" onchange="updatePatternSetting('SIDE4STARTX', this.value)">
                            <label for="side4StartY">Y:</label>
                            <input type="number" id="side4StartY" class="setting-input" min="0" max="37" placeholder="0.0" onchange="updatePatternSetting('SIDE4STARTY', this.value)">
                        </div>
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Shift Distance</h3>
                        <input type="number" id="side4ShiftX" class="setting-input" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE4SHIFTX', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Sweep Distance</h3>
                        <input type="number" id="side4SweepY" class="setting-input" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE4SWEEPY', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Shift Speed</h3>
                        <input type="number" id="side4PaintingXSpeed" class="setting-input" max="20000" step="1000" placeholder="20" onchange="updatePatternSetting('SIDE4PAINTINGXSPEED', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Sweep Speed</h3>
                        <input type="number" id="side4PaintingYSpeed" class="setting-input" max="20000" step="1000" placeholder="20" onchange="updatePatternSetting('SIDE4PAINTINGYSPEED', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Z Height</h3>
                        <input type="number" id="side4ZHeight" class="setting-input" min="-2.5" max="0" step="0.1" placeholder="0.0" onchange="updatePatternSetting('SIDE4ZHEIGHT', this.value)">
                    </div>
                    <div class="pattern-setting-group">
                        <h3>Servo Angle</h3>
                        <input type="number" id="servoAngleSide4" class="setting-input" min="30" max="100" step="1" placeholder="0" onchange="updatePatternSetting('SERVO_ANGLE_SIDE4', this.value)">
                    </div>
                </div>
            </div>
            
            <div class="main-divider"></div>
            
            <!-- Settings Controls -->
            <div class="settings-controls">
                <!-- Remove Load and Save buttons -->
                <!-- <button class="main-btn blue" onclick="loadPatternSettings()">Load Settings</button> -->
                <!-- <button class="main-btn highlight" onclick="savePatternSettings()">Save Settings</button> -->
            </div>
        </div> <!-- End of pattern-settings-content-wrapper -->
    </div>
    
    <!-- PNP Motion Settings Section -->
    <div class="pattern-settings-container">
        <!-- Clickable Header for Toggling -->
        <h2 id="pnpSettingsHeader" class="pattern-settings-header">
            PNP Motion Settings
        </h2>

        <!-- Content Wrapper for Collapsing -->
        <div class="pattern-settings-content-wrapper" id="pnpSettingsContent">
            <div class="pattern-settings-grid">
                <div class="pattern-setting-group">
                    <h3>X Axis</h3>
                    <div class="setting-inputs labeled-inputs">
                        <label for="pnp_x_speed">Speed (steps/s):</label>
                        <input type="number" id="pnp_x_speed" class="setting-input" min="1000" max="30000" step="500" placeholder="10000">
                        <label for="pnp_x_accel">Accel (steps/s²):</label>
                        <input type="number" id="pnp_x_accel" class="setting-input" min="1000" max="30000" step="500" placeholder="10000">
                    </div>
                </div>
                <div class="pattern-setting-group">
                    <h3>Y Axis</h3>
                    <div class="setting-inputs labeled-inputs">
                        <label for="pnp_y_speed">Speed (steps/s):</label>
                        <input type="number" id="pnp_y_speed" class="setting-input" min="1000" max="35000" step="500" placeholder="25000">
                        <label for="pnp_y_accel">Accel (steps/s²):</label>
                        <input type="number" id="pnp_y_accel" class="setting-input" min="1000" max="35000" step="500" placeholder="32000">
                    </div>
                </div>
            </div>
            
            <div class="main-divider"></div>
            
            <!-- Settings Controls -->
            <div class="settings-controls">
                <button class="main-btn" onclick="updatePnpSettings()">Update PNP Settings</button>
            </div>
        </div>
    </div>
</body>
</html>
)rawliteral";

#endif // HTML_CONTENT_H
