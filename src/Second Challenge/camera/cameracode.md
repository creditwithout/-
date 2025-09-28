```py
# OpenMV H7 Computer Vision System for Autonomous Robot Navigation
# Advanced blob detection, tracking, and obstacle avoidance implementation
# This system provides real-time visual processing for robot navigation decisions
# Author: SETEKI 
# Version: 2.0 - Enhanced tracking and state machine implementation

import sensor      # Core camera sensor interface for image capture and processing
import time        # Time utilities for timing operations and delays
import pyb         # MicroPython board utilities for hardware control
import math        # Mathematical functions for geometric calculations
from pyb import LED # LED control interface for status indication

# =================================================================
# =========== MANUAL CONFIGURATION SECTION - 100% CUSTOMIZABLE ====
# =================================================================
# This section contains all configurable parameters for the vision system
# Each parameter can be tuned for optimal performance in different environments
# All values are empirically determined and can be adjusted based on testing

# --- 1. CAMERA SENSOR AND WINDOWING CONFIGURATION ---
# High definition sensor resolution provides maximum detail for blob detection
# HD resolution offers superior precision for obstacle identification at distance
SENSOR_RESOLUTION = sensor.HD  # High Definition: 1280x720 pixels for maximum precision
PIXEL_FORMAT = sensor.RGB565   # 16-bit color format balancing quality and processing speed
# Manual windowing crops the image to focus on the road surface area
# Format: (x_start, y_start, width, height) - focuses on lower portion of image
MANUAL_WINDOWING = (0, 300, 1280, 300)  # Crop to lower 300 pixels for road surface focus

# --- 2. OPTICAL LENS CORRECTION PARAMETERS ---
# Lens correction compensates for optical distortion in the camera lens
# Disabled by default but can be enabled for wide-angle lenses with distortion
ENABLE_LENS_CORRECTION = False  # Set to True if lens distortion correction is needed
LENS_STRENGTH = 1.8             # Correction strength factor (higher = more correction)
LENS_ZOOM = 1.0                 # Zoom factor for lens correction (1.0 = no zoom)

# --- 3. COLOR BLOB DETECTION THRESHOLDS AND PARAMETERS ---
# LAB color space thresholds for robust color detection under varying lighting
# LAB space provides better color consistency across different illumination conditions
# Format: [L_min, L_max, A_min, A_max, B_min, B_max] - L=lightness, A=green-red, B=blue-yellow
threshold_color_1 = (0, 100, 4, 127, -124, 127)    # Color 1: Red/Orange obstacles
threshold_color_2 = (0, 100, -128, -18, 11, 127)   # Color 2: Green obstacles  
threshold_color_3 = (0, 100, 8, 127, -128, 8)      # Color 3: Blue lane markers
color_thresholds = [threshold_color_1, threshold_color_2, threshold_color_3]
# Visualization colors for blob drawing (RGB format for display purposes)
blob_draw_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # Red, Green, Blue

# Blob detection filtering parameters to eliminate noise and false positives
BLOB_PIXELS_THRESHOLD = 1000   # Minimum pixel count for valid blob detection
BLOB_AREA_THRESHOLD = 1000     # Minimum area in pixels for blob validation

# Shape density filter to ensure detected blobs have solid, consistent shapes
# This eliminates fragmented or scattered pixel groups that aren't real obstacles
ENABLE_DENSITY_FILTER = True   # Enable density-based blob filtering
BLOB_DENSITY_THRESHOLD = 0.8   # Minimum density ratio (solid pixels / total area)

# --- 4. STATE MACHINE AND AVOIDANCE LOGIC CONFIGURATION ---
# Initial camera state for system startup and operation mode
ESTADO_INICIAL_CAMARA = "NORMAL_STREAMING"  # Default to continuous streaming mode
# Y-coordinate threshold for determining if obstacles are close enough to require avoidance
Y_PIXEL_THRESHOLD_FOR_CHECK = 125  # Pixels from bottom - closer obstacles trigger avoidance
# Duration for initial obstacle assessment phase in milliseconds
CHECK_STATE_DURATION_MS = 2000      # 2-second analysis period for obstacle counting
# Confidence threshold for lane marker detection during check phases
CHECK_CONFIDENCE_THRESHOLD = 0.4    # 40% confidence required for lane marker detection
# Confidence threshold for close obstacle detection during normal operation
CLOSE_BLOB_CONFIDENCE_THRESHOLD = 0.7  # 70% confidence for close obstacle detection

# --- 5. UART COMMUNICATION INTERFACE CONFIGURATION ---
# Serial communication setup for data transmission to main robot controller
# UART3 provides high-speed communication with 115200 baud rate
uart = pyb.UART(3, 115200, timeout_char=1000)  # 1-second timeout for robust communication

# --- 6. STATUS INDICATION LED CONFIGURATION ---
# Hardware LED control for visual status indication and debugging
led_red = LED(1)    # Red LED for error states and red blob detection
led_green = LED(2)  # Green LED for normal operation and green blob detection  
led_blue = LED(3)   # Blue LED for special states and blue lane marker detection

# --- 7. ADVANCED BLOB TRACKING SYSTEM PARAMETERS ---
# Multi-object tracking system for persistent obstacle monitoring
tracked_blobs = []                    # List of currently tracked blob objects
next_blob_id = 0                     # Unique identifier counter for new blob assignments
MATCHING_DISTANCE_THRESHOLD = 600    # Maximum pixel distance for blob association
FRAMES_UNSEEN_THRESHOLD = 15         # Frames before considering a blob lost

# =================================================================
# ==================== CONFIGURATION SECTION COMPLETE =============
# =================================================================

# =================================================================
# =========== AUXILIARY FUNCTIONS AND UTILITY CLASSES =============
# =================================================================
# These functions provide geometric calculations and coordinate transformations
# for blob position analysis and region-of-interest (ROI) determination
# The ROI system divides the camera view into strategic zones for navigation decisions

def get_section_from_coords(cx, cy, width):
    """
    Convert blob coordinates to a standardized section number for UART transmission.
    
    This function transforms 2D blob coordinates into a single section identifier
    that represents both horizontal (left/center/right) and vertical (upper/lower)
    position information. The section numbering follows a consistent 25-50-25
    horizontal division pattern for navigation coherence.
    
    Args:
        cx (int): Center X coordinate of the detected blob in pixels
        cy (int): Center Y coordinate of the detected blob in pixels  
        width (int): Total image width in pixels for coordinate normalization
        
    Returns:
        int: Section number (1-6) representing the blob's position:
             - Sections 1-3: Upper row (left, center, right)
             - Sections 4-6: Lower row (left, center, right)
    """
    col = get_roi_from_cx(cx, width)  # Use consistent 25-50-25 logic for coherence
    row = cy // (sensor.height() // 2)  # Determine upper (0) or lower (1) row
    return int((row * 3) + col + 1)  # Convert to 1-based section numbering

def get_roi_from_cx(cx, width):
    """
    Determine Region of Interest (ROI) based on blob's horizontal position.
    
    This function implements a strategic 25-50-25 horizontal division of the
    camera view. This division provides optimal coverage for navigation decisions:
    - Left/Right zones (25% each) for lateral obstacle detection
    - Center zone (50%) for primary navigation path monitoring
    
    The ROI system enables different avoidance strategies based on obstacle
    position, allowing the robot to make context-aware navigation decisions.
    
    Args:
        cx (int): Center X coordinate of the detected blob in pixels
        width (int): Total image width in pixels for boundary calculations
        
    Returns:
        int: ROI index representing horizontal position:
             - 0: Left region (0% to 25% of image width)
             - 1: Center region (25% to 75% of image width) 
             - 2: Right region (75% to 100% of image width)
    """
    # Strategic division: Laterals 25% each, Center 50% for optimal navigation coverage
    if cx < width / 4:
        return 0  # Left region - obstacles here require left avoidance maneuvers
    elif cx < (width * 3) / 4:
        return 1  # Center region - obstacles here require primary avoidance strategies
    else:
        return 2  # Right region - obstacles here require right avoidance maneuvers

# =================================================================
# =========== CAMERA SENSOR INITIALIZATION AND CONFIGURATION ======
# =================================================================
# Complete camera sensor setup with optimal parameters for vision processing
# This initialization sequence ensures consistent image quality and processing speed
# All settings are configured for maximum blob detection accuracy and reliability

# Reset camera sensor to default state and clear any previous configuration
sensor.reset()
# Set pixel format to RGB565 for optimal balance between color accuracy and processing speed
sensor.set_pixformat(PIXEL_FORMAT)
# Configure sensor resolution to HD for maximum detail and precision in blob detection
sensor.set_framesize(SENSOR_RESOLUTION)
# Apply manual windowing to crop image and focus on relevant road surface area
if MANUAL_WINDOWING: sensor.set_windowing(MANUAL_WINDOWING)
# Skip initial frames to allow sensor stabilization and achieve consistent exposure
sensor.skip_frames(time=2000)  # 2-second stabilization period for optimal image quality
# Disable automatic gain control to maintain consistent blob detection thresholds
sensor.set_auto_gain(False)    # Manual gain control prevents exposure variations
# Disable automatic white balance to ensure consistent color detection across lighting
sensor.set_auto_whitebal(False)  # Manual white balance maintains color threshold reliability
# Initialize timing clock for frame rate monitoring and performance analysis
clock = time.clock()

# =================================================================
# =========== GLOBAL STATE VARIABLES AND SYSTEM STATUS ============
# =================================================================
# These variables manage the system's operational state and tracking information
# State management enables different processing modes and communication protocols

# Camera operational state - determines current processing mode and behavior
estado_camara = ESTADO_INICIAL_CAMARA  # Initialize to normal streaming mode
# Maximum number of obstacles that can be avoided in current navigation segment
bloques_a_esquivar_max = 0  # Starts at 0, defined during CHECK phases based on analysis
# Current count of obstacles successfully avoided in this navigation segment
bloques_esquivados_actualmente = 0  # Tracks avoidance progress for limit enforcement
# Flag controlling UART data transmission to main robot controller
streaming_habilitado = False  # Disabled until CHECK phase completion enables streaming
# Type of CHECK analysis being performed (INITIAL or NORMAL)
check_type = ""  # Determines analysis methodology and decision criteria
# ROI masking mode for selective vision processing (FULL, IGNORE_LEFT, IGNORE_RIGHT)
roi_mask_mode = "FULL"  # NEW: Controls vision mask for directional obstacle filtering

print("Camera configuration completed. Initializing multifunction main loop...")

# =================================================================
# =========== SYSTEM STARTUP INDICATION AND STATUS SEQUENCE =======
# =================================================================
# Visual startup sequence provides immediate feedback about system initialization
# LED patterns indicate successful boot and readiness for operation

# Illuminate all LEDs simultaneously to create white light indicating script execution
# This provides immediate visual confirmation that the system is starting up
led_red.on()    # Red LED component for white light indication
led_green.on()  # Green LED component for white light indication
led_blue.on()   # Blue LED component for white light indication
time.sleep_ms(1000)  # 1-second white light display for startup confirmation
# Turn off all LEDs to prepare for operational status indication
led_red.off()
led_green.off()
led_blue.off()

# Green LED blinking sequence indicates successful initialization completion
# Three rapid blinks provide clear visual confirmation of proper startup
for _ in range(3):
    led_green.on()    # Illuminate green LED for successful status indication
    time.sleep_ms(100)  # 100ms ON duration for visible blink
    led_green.off()   # Turn off green LED
    time.sleep_ms(100)  # 100ms OFF duration for clear blink separation

# =================================================================
# =========== MAIN PROCESSING LOOP - CONTINUOUS VISION OPERATION ====
# =================================================================
# The main loop provides continuous image processing and communication handling
# This loop manages multiple operational states and real-time decision making
# All vision processing, blob tracking, and UART communication occurs here

while True:
    # Update frame timing for performance monitoring and frame rate calculation
    clock.tick()  # Increment frame counter and calculate processing timing
    
    # Default LED status indication - green LED shows normal operation
    # LED states provide immediate visual feedback about system status
    led_red.off()    # Red LED off during normal operation
    led_green.on()   # Green LED on indicates system running normally
    led_blue.off()   # Blue LED off during normal operation

    # =================================================================
    # =========== UART COMMUNICATION PROTOCOL HANDLING =================
    # =================================================================
    # Process incoming commands from main robot controller via UART
    # Command protocol enables dynamic system behavior and state transitions
    # Each command triggers specific processing modes or parameter changes
    
    if uart.any():  # Check for incoming UART data availability
        comando = uart.read(1)  # Read single byte command from robot controller
        print(f"UART data received: {comando}")
        
        # Command '2': Initialize INITIAL CHECK phase for obstacle assessment
        if comando == b'2':
            print("Command '2' received. Preparing INITIAL CHECK.")
            estado_camara = "CHECK"      # Switch to CHECK processing mode
            check_type = "INITIAL"       # Set analysis type to initial assessment
            
        # Command '1': Initialize NORMAL CHECK phase for ongoing obstacle monitoring
        elif comando == b'1':
            print("Command '1' received. Preparing NORMAL CHECK.")
            estado_camara = "CHECK"      # Switch to CHECK processing mode
            check_type = "NORMAL"        # Set analysis type to normal monitoring
            
        # Command '3': Activate wall confirmation analysis for navigation decisions
        elif comando == b'3':
            print("Command '3' received. Preparing CONFIRM WALL.")
            estado_camara = "CONFIRM_WALL"  # Switch to wall confirmation mode
            
        # Command 'R': Set ROI mask to ignore left side obstacles (right turn preparation)
        elif comando == b'R':
            print("Command 'R' received. MASK: Ignoring Left side.")
            roi_mask_mode = "IGNORE_LEFT"  # Mask left region for right-turn scenarios
            
        # Command 'L': Set ROI mask to ignore right side obstacles (left turn preparation)  
        elif comando == b'L':
            print("Command 'L' received. MASK: Ignoring Right side.")
            roi_mask_mode = "IGNORE_RIGHT"  # Mask right region for left-turn scenarios
            
        # Command 'N': Reset ROI mask to full vision (normal navigation mode)
        elif comando == b'N':
            print("Command 'N' received. MASK: Full Vision enabled.")
            roi_mask_mode = "FULL"  # Enable full field of view for normal operation

    # =================================================================
    # =========== WALL CONFIRMATION STATE - OBSTACLE TYPE ANALYSIS =====
    # =================================================================
    # This state performs single-frame analysis to distinguish between walls and obstacles
    # Critical for navigation decisions - determines if robot should turn or continue straight
    # Analysis focuses on center region to avoid false positives from lateral obstacles
    
    if estado_camara == "CONFIRM_WALL":
        # LED indication: Yellow (red+green) indicates wall confirmation analysis in progress
        led_red.on()    # Red LED component for yellow indication
        led_green.on()  # Green LED component for yellow indication  
        led_blue.off()  # Blue LED off during wall confirmation

        print("Analyzing image for wall confirmation...")
        # Capture single frame for wall confirmation analysis
        img = sensor.snapshot()

        # Initialize detection flag for center region obstacle analysis
        blob_found_in_center = False
        
        # Search for obstacle blobs (colors 0 and 1) across entire image
        # These colors represent red and green obstacles that require avoidance
        for i in range(2):  # Analyze first two color thresholds (obstacle colors)
            # Find all blobs matching current color threshold with filtering applied
            all_blobs = img.find_blobs([color_thresholds[i]], 
                                     pixels_threshold=BLOB_PIXELS_THRESHOLD, 
                                     area_threshold=BLOB_AREA_THRESHOLD, 
                                     merge=True)  # Merge overlapping detections
            
            # Apply density filtering to eliminate fragmented or scattered pixel groups
            blobs_found = [b for b in all_blobs if b.density() > BLOB_DENSITY_THRESHOLD] if ENABLE_DENSITY_FILTER else all_blobs

            # Filter blobs to only those located in the center region (ROI index 1)
            # Center region analysis provides most reliable wall vs obstacle distinction
            for b in blobs_found:
                if get_roi_from_cx(b.cx(), img.width()) == 1:  # 1 is center region index
                    blob_found_in_center = True
                    # Draw red rectangle around detected obstacle for visual confirmation
                    img.draw_rectangle(b.rect(), color=(255, 0, 0))  # Red visualization
                    print(f"Obstacle blob found in center section! Color index: {i}")
                    break  # Exit blob analysis loop once center obstacle found
            if blob_found_in_center:
                break  # Exit color analysis loop once obstacle confirmed

        # Decision logic: Send appropriate response based on center region analysis
        if blob_found_in_center:
            # Obstacle detected in center - this is NOT a wall, robot must avoid
            print("Result: Close obstacle detected. Sending 'P,8'.")
            uart.write("P,8\r\n")  # Signal: Obstacle requiring avoidance maneuver
        else:
            # No obstacles in center region - this is a wall or distant obstacle
            print("Result: Wall detected (or distant obstacle ignored). Sending 'P,9'.")
            uart.write("P,9\r\n")  # Signal: Wall - robot can proceed or turn

        # Return to normal streaming state and restart loop to avoid further processing
        estado_camara = "NORMAL_STREAMING"
        continue  # Skip remaining processing for this frame

    # =================================================================
    # =========== CHECK STATE - OBSTACLE ASSESSMENT AND ANALYSIS ========
    # =================================================================
    # CHECK state performs comprehensive obstacle analysis to determine avoidance limits
    # Two types of CHECK analysis: INITIAL (startup assessment) and NORMAL (ongoing monitoring)
    # Analysis results determine maximum number of obstacles robot can avoid in current segment
    
    if estado_camara == "CHECK":
        # =================================================================
        # =========== INITIAL CHECK - STARTUP OBSTACLE ASSESSMENT ==========
        # =================================================================
        # INITIAL CHECK analyzes the environment at robot startup to set avoidance limits
        # This prevents the robot from attempting to avoid too many obstacles in unknown terrain
        # Analysis focuses on center region to identify immediate navigation challenges
        
        if check_type == "INITIAL":
            # LED indication: Blue LED indicates INITIAL CHECK analysis in progress
            led_blue.on()   # Blue LED for INITIAL CHECK status indication
            led_green.off() # Green LED off during CHECK analysis

            print(f"Starting INITIAL analysis for {CHECK_STATE_DURATION_MS} ms...")
            # Initialize timing for analysis duration control
            start_time = time.ticks_ms()
            # Flag to track if any obstacles are detected during analysis period
            initial_blob_found = False
            # Time-based analysis loop - runs for specified duration to assess environment
            while time.ticks_diff(time.ticks_ms(), start_time) < CHECK_STATE_DURATION_MS:
                # Capture current frame for obstacle analysis
                img = sensor.snapshot()
                
                # Search for obstacle blobs (colors 0 and 1) representing red and green obstacles
                # These are the primary obstacles that require avoidance maneuvers
                for i in range(2):  # Analyze first two color thresholds (obstacle colors)
                    # Find all blobs matching current color with filtering parameters
                    all_blobs = img.find_blobs([color_thresholds[i]], 
                                             pixels_threshold=BLOB_PIXELS_THRESHOLD, 
                                             area_threshold=BLOB_AREA_THRESHOLD, 
                                             merge=True)  # Merge overlapping detections
                    
                    # Apply density filtering to ensure detected blobs are solid obstacles
                    blobs_found = [b for b in all_blobs if b.density() > BLOB_DENSITY_THRESHOLD] if ENABLE_DENSITY_FILTER else all_blobs
                    
                    # Filter blobs to only those in center region for reliable obstacle assessment
                    # Center region provides most accurate indication of immediate navigation challenges
                    for b in blobs_found:
                        if get_roi_from_cx(b.cx(), img.width()) == 1:  # Center region check
                            initial_blob_found = True
                            break  # Exit blob analysis once obstacle found
                    if initial_blob_found:
                        break  # Exit color analysis loop once obstacle confirmed

            print("INITIAL analysis completed.")
            
            # Set avoidance limit based on analysis results
            if initial_blob_found:
                # Obstacle detected - allow maximum 1 avoidance maneuver for safety
                bloques_a_esquivar_max = 1
                print("INITIAL CHECK result: Will avoid MAXIMUM 1 obstacle.")
            else:
                # No obstacles detected - no avoidance maneuvers needed
                bloques_a_esquivar_max = 0
                print("INITIAL CHECK result: Will avoid 0 obstacles.")

            # Send response to robot controller in format: C,{lane_markers}
            # For INITIAL CHECK, lane markers don't apply, so send 0
            respuesta = "C,0\r\n"
            uart.write(respuesta)
            print(f"Sending INITIAL CHECK: {respuesta.strip()} (Internal limit: {bloques_a_esquivar_max})")
            
            # Enable UART data streaming after successful CHECK completion
            streaming_habilitado = True
            print("UART data streaming ENABLED.")

        # =================================================================
        # =========== NORMAL CHECK - ONGOING OBSTACLE MONITORING ===========
        # =================================================================
        # NORMAL CHECK performs comprehensive analysis during robot operation
        # This analysis determines both obstacle avoidance limits and lane marker presence
        # More sophisticated than INITIAL CHECK, includes confidence-based decision making
        
        elif check_type == "NORMAL":
            # LED indication: Green LED indicates NORMAL CHECK analysis in progress
            led_green.on()  # Green LED for NORMAL CHECK status indication

            # =================================================================
            # =========== ROBUST ANALYSIS LOGIC FOR NORMAL OPERATION ==========
            # =================================================================
            # Initialize timing and analysis counters for comprehensive assessment
            start_time = time.ticks_ms()
            frames_analyzed = 0           # Total frames processed during analysis
            third_color_detections = 0    # Count of blue lane marker detections
            close_blob_detections = 0     # Count of close obstacle detections

            print(f"Starting NORMAL analysis for {CHECK_STATE_DURATION_MS} ms...")

            while time.ticks_diff(time.ticks_ms(), start_time) < CHECK_STATE_DURATION_MS:
                img = sensor.snapshot()
                frames_analyzed += 1
                frame_had_close_blob = False

                # 1. Analizar si hay bloques CERCANOS (colores 1 y 2) en la sección central
                for i in range(2):
                    all_blobs = img.find_blobs([color_thresholds[i]], pixels_threshold=BLOB_PIXELS_THRESHOLD, area_threshold=BLOB_AREA_THRESHOLD, merge=True)
                    blobs_found = [b for b in all_blobs if b.density() > BLOB_DENSITY_THRESHOLD] if ENABLE_DENSITY_FILTER else all_blobs
                    for b in blobs_found:
                        # Comprobar si está cerca Y en el centro
                        if (b.y() + b.h()) > Y_PIXEL_THRESHOLD_FOR_CHECK and get_roi_from_cx(b.cx(), img.width()) == 1:
                            frame_had_close_blob = True
                            break
                    if frame_had_close_blob:
                        break

                if frame_had_close_blob:
                    close_blob_detections += 1

                # 2. Contar si el tercer color está presente en la sección central
                all_blobs_c3 = img.find_blobs([color_thresholds[2]], pixels_threshold=BLOB_PIXELS_THRESHOLD, area_threshold=BLOB_AREA_THRESHOLD, merge=True)
                # Filtrar por densidad Y por estar en el centro
                blobs_in_center_c3 = [b for b in all_blobs_c3 if get_roi_from_cx(b.cx(), img.width()) == 1]
                if any(b.density() > BLOB_DENSITY_THRESHOLD for b in blobs_in_center_c3) if ENABLE_DENSITY_FILTER else blobs_in_center_c3:
                    third_color_detections += 1

            print(f"Análisis NORMAL completado. Detecciones Bloque Cercano: {close_blob_detections}.")

            # --- TOMAR DECISIONES BASADAS EN EL ANÁLISIS ---
            close_blob_confidence = close_blob_detections / frames_analyzed if frames_analyzed > 0 else 0
            if close_blob_confidence > CLOSE_BLOB_CONFIDENCE_THRESHOLD:
                bloques_a_esquivar_max = 2
                print(f"Resultado CHECK NORMAL: Se esquivarán 2 (Confianza: {close_blob_confidence:.2f}).")
            else:
                bloques_a_esquivar_max = 1
                print(f"Resultado CHECK NORMAL: Se esquivará 1 (Confianza: {close_blob_confidence:.2f}).")

            third_color_confidence = third_color_detections / frames_analyzed if frames_analyzed > 0 else 0
            tercer_umbral_presente = third_color_confidence > CHECK_CONFIDENCE_THRESHOLD
            if tercer_umbral_presente:
                print(f"Resultado CHECK: Marcas de carril presentes (Confianza: {third_color_confidence:.2f})")
            else:
                print(f"Resultado CHECK: Marcas de carril ausentes (Confianza: {third_color_confidence:.2f})")

            marcas_presentes_int = 1 if tercer_umbral_presente else 0
            respuesta = f"C,{marcas_presentes_int}\r\n"
            uart.write(respuesta)
            print(f"Enviando CHECK NORMAL: {respuesta.strip()} (Límite interno: {bloques_a_esquivar_max})")

        # Reiniciar variables para el siguiente tramo
        bloques_esquivados_actualmente = 0
        tracked_blobs = []
        estado_camara = "NORMAL_STREAMING"
        continue

    # --- LÓGICA DE PROCESAMIENTO DE IMAGEN Y VISUALIZACIÓN (SIEMPRE ACTIVA) ---
    img = sensor.snapshot()
    if ENABLE_LENS_CORRECTION:
        img.lens_corr(strength=LENS_STRENGTH, zoom=LENS_ZOOM)

    # =================================================================
    # =========== ADVANCED BLOB TRACKING SYSTEM WITH PREDICTION ==========
    # =================================================================
    # Multi-object tracking system with velocity prediction and association
    # Implements simplified Kalman filter approach for robust obstacle tracking
    # System maintains persistent tracking across frames for reliable navigation decisions

    # =================================================================
    # =========== STEP 1: VELOCITY-BASED POSITION PREDICTION ============
    # =================================================================
    # Predict new positions of currently tracked blobs based on their velocity vectors
    # This prediction improves association accuracy and handles temporary occlusions
    # Prediction step occurs before new blob detection for optimal matching
    
    for tb in tracked_blobs:
        # Update predicted position using stored velocity components
        tb['cx'] += tb['vx']  # Predict new X coordinate based on X velocity
        tb['cy'] += tb['vy']  # Predict new Y coordinate based on Y velocity
        # Note: Rectangle coordinates are only updated on actual detection matches
        # This keeps prediction lightweight while maintaining tracking accuracy

    # =================================================================
    # =========== STEP 2: ADAPTIVE BLOB DETECTION WITH ROI FILTERING ======
    # =================================================================
    # Detect new blobs in current frame with adaptive region-of-interest filtering
    # ROI masking allows selective obstacle detection based on navigation context
    # Detection focuses on obstacle colors (red and green) for avoidance decisions
    
    # Initialize full image search region
    search_roi = (0, 0, img.width(), img.height())
    img_width = img.width()
    img_height = img.height()

    # Apply ROI masking based on navigation context for selective detection
    if roi_mask_mode == "IGNORE_LEFT":
        # Mask left region for right-turn preparation scenarios
        roi_x = img_width // 4
        search_roi = (roi_x, 0, img_width - roi_x, img_height)
        # Draw yellow debug line to visualize masked region
        img.draw_line(roi_x, 0, roi_x, img_height, color=(255, 255, 0))
    elif roi_mask_mode == "IGNORE_RIGHT":
        # Mask right region for left-turn preparation scenarios
        roi_w = (img_width * 3) // 4
        search_roi = (0, 0, roi_w, img_height)
        # Draw yellow debug line to visualize masked region
        img.draw_line(roi_w, 0, roi_w, img_height, color=(255, 255, 0))

    # Initialize detection storage for current frame analysis
    all_detections_this_frame = []
    
    # Detect blobs for obstacle colors only (colors 0 and 1: red and green)
    for i, current_threshold in enumerate(color_thresholds[:2]):  # Only obstacle colors
        # Find blobs matching current color threshold with filtering parameters
        blobs_of_color = img.find_blobs([current_threshold], 
                                      pixels_threshold=BLOB_PIXELS_THRESHOLD, 
                                      area_threshold=BLOB_AREA_THRESHOLD, 
                                      merge=True,      # Merge overlapping detections
                                      margin=10,       # Margin for blob merging
                                      roi=search_roi)  # Apply ROI filtering

        # Apply density filtering to eliminate fragmented or scattered pixel groups
        filtered_blobs = [b for b in blobs_of_color if b.density() > BLOB_DENSITY_THRESHOLD] if ENABLE_DENSITY_FILTER else blobs_of_color

        # Store detected blobs with color index for tracking association
        for blob in filtered_blobs:
            all_detections_this_frame.append({'blob': blob, 'color_index': i})

    # =================================================================
    # =========== STEP 3: NEAREST NEIGHBOR ASSOCIATION AND UPDATE ========
    # =================================================================
    # Associate detected blobs with existing tracked blobs using distance-based matching
    # Implements simplified Kalman filter correction for velocity estimation
    # Association prevents duplicate tracking and maintains blob identity across frames
    
    # Reset matching flags for all tracked blobs
    for tracked_blob in tracked_blobs:
        tracked_blob['matched'] = False

    # Perform nearest neighbor association for each tracked blob
    for tracked_blob in tracked_blobs:
        best_match = None
        min_dist_sq = MATCHING_DISTANCE_THRESHOLD**2  # Maximum association distance squared

        # Find closest matching detection of same color type
        for detection in all_detections_this_frame:
            # Match only blobs of same color and not already matched
            if tracked_blob['color_index'] == detection['color_index'] and not detection.get('matched', False):
                # Calculate distance squared between predicted position and actual detection
                # Distance calculation uses PREDICTED position for improved association accuracy
                dist_sq = (tracked_blob['cx'] - detection['blob'].cx())**2 + (tracked_blob['cy'] - detection['blob'].cy())**2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    best_match = detection

        # Update tracked blob with best match if found
        if best_match:
            new_blob = best_match['blob']
            
            # Velocity correction using simplified Kalman filter approach
            # Correct velocity based on difference between prediction and actual detection
            # This is a simplified form of Kalman filter correction for smooth tracking
            # Factor of 0.5 provides balanced tracking with smooth velocity updates
            tracked_blob['vx'] = int((tracked_blob['vx'] * 0.5) + ((new_blob.cx() - tracked_blob['cx']) * 0.5))
            tracked_blob['vy'] = int((tracked_blob['vy'] * 0.5) + ((new_blob.cy() - tracked_blob['cy']) * 0.5))

            # Update position to actual detected coordinates
            tracked_blob['cx'] = new_blob.cx()      # Update X coordinate to detected position
            tracked_blob['cy'] = new_blob.cy()      # Update Y coordinate to detected position
            tracked_blob['rect'] = new_blob.rect()  # Update bounding rectangle
            tracked_blob['frames_unseen'] = 0       # Reset unseen frame counter
            tracked_blob['matched'] = True          # Mark as successfully matched
            best_match['matched'] = True            # Mark detection as used

    # =================================================================
    # =========== STEP 4: LOST BLOB MANAGEMENT AND CLEANUP ===============
    # =================================================================
    # Manage tracked blobs that were not detected in current frame
    # Remove blobs that are lost (too many unseen frames) or have passed the robot
    # Count successfully avoided obstacles for navigation limit enforcement
    
    blobs_to_remove = []
    for tracked_blob in tracked_blobs:
        # Increment unseen counter for unmatched blobs
        if not tracked_blob['matched']:
            tracked_blob['frames_unseen'] += 1

        # Determine if blob should be removed from tracking
        is_lost = tracked_blob['frames_unseen'] > FRAMES_UNSEEN_THRESHOLD  # Too many unseen frames
        is_passed = tracked_blob['cy'] > (sensor.height() * 0.9)          # Blob passed robot (bottom 10%)

        # Remove blob if lost or passed, and count as avoided obstacle
        if is_lost or is_passed:
            blobs_to_remove.append(tracked_blob)
            # Count obstacle avoidance for navigation limit tracking
            if tracked_blob['color_index'] < 2:  # Only count obstacle colors (0, 1)
                bloques_esquivados_actualmente += 1
                print(f"OBSTACLE AVOIDED (ID: {tracked_blob['id']}). Total: {bloques_esquivados_actualmente}")

    # Remove lost/passed blobs from tracking list
    if blobs_to_remove:
        tracked_blobs = [b for b in tracked_blobs if b not in blobs_to_remove]

    # =================================================================
    # =========== STEP 5: NEW BLOB INITIALIZATION AND TRACKING ==========
    # =================================================================
    # Create new tracked blob objects for unmatched detections
    # Initialize new blobs with zero velocity and assign unique IDs
    # This handles newly appeared obstacles that weren't previously tracked
    
    for detection in all_detections_this_frame:
        # Process only detections that weren't matched to existing tracked blobs
        if not detection.get('matched', False):
            new_blob = detection['blob']
            # Create new tracked blob object with initial parameters
            new_tracked_blob = {
                'id': next_blob_id,                    # Assign unique identifier
                'cx': new_blob.cx(),                   # Set initial X coordinate
                'cy': new_blob.cy(),                   # Set initial Y coordinate
                'vx': 0,                              # Initialize X velocity to zero
                'vy': 0,                              # Initialize Y velocity to zero
                'rect': new_blob.rect(),              # Store bounding rectangle
                'color_index': detection['color_index'], # Store color classification
                'matched': True,                      # Mark as matched in this frame
                'frames_unseen': 0                    # Initialize unseen counter
            }
            tracked_blobs.append(new_tracked_blob)     # Add to tracking list
            next_blob_id += 1                          # Increment ID counter for next blob

    # =================================================================
    # =========== BLOB TRACKING SYSTEM COMPLETION =====================
    # =================================================================
    # Advanced multi-object tracking system now complete
    # System provides persistent obstacle tracking with velocity prediction
    # Ready for navigation decision making and UART communication

    # =================================================================
    # =========== CLOSEST OBSTACLE SELECTION FOR NAVIGATION ==============
    # =================================================================
    # Select the closest obstacle blob for navigation decision making
    # Closest blob is determined by lowest Y coordinate (bottom of bounding box)
    # Only obstacle colors (0, 1) are considered for avoidance decisions
    
    # Filter tracked blobs to only obstacle colors (red and green)
    candidate_blobs = [b for b in tracked_blobs if b['color_index'] < 2]
    closest_blob = None
    
    # Find closest obstacle based on bottom edge position (Y + height)
    if candidate_blobs:
        # Select blob with maximum bottom edge position (closest to robot)
        closest_blob = max(candidate_blobs, key=lambda b: b['rect'][1] + b['rect'][3])

    # =================================================================
    # =========== VISUALIZATION AND DEBUG DISPLAY ======================
    # =================================================================
    # Draw visual indicators for all tracked blobs with status information
    # Provides real-time visual feedback for debugging and monitoring
    
    for tb in tracked_blobs:
        # Draw colored rectangle around each tracked blob based on color classification
        img.draw_rectangle(tb['rect'], color=blob_draw_colors[tb['color_index']])
        # Display blob ID above the rectangle for tracking identification
        img.draw_string(tb['rect'][0], tb['rect'][1] - 10, f"ID:{tb['id']}", color=(255,255,0))
        # Show unseen frame count for unmatched blobs (red text)
        if not tb['matched']:
            img.draw_string(tb['rect'][0], tb['rect'][1] + 10, f"U:{tb['frames_unseen']}", color=(255,0,0))

    # =================================================================
    # =========== NAVIGATION DECISION LOGIC AND UART MESSAGE GENERATION ===
    # =================================================================
    # Determine if closest obstacle should be reported to robot controller
    # Apply avoidance limits and color-based reporting rules for navigation decisions
    # Generate UART message with obstacle position and region information
    
    # Initialize default UART message values (no obstacle detected)
    blob_sect_str = "0"  # Default: no obstacle section
    blob_roi_str = "0"   # Default: no obstacle region

    # Check if obstacle should be reported based on avoidance limits and detection
    if bloques_esquivados_actualmente < bloques_a_esquivar_max and closest_blob:
        # Calculate ROI (0, 1, 2) and color index for avoidance logic
        roi = get_roi_from_cx(closest_blob['cx'], img.width())
        color_index = closest_blob['color_index']

        # Apply color-based reporting rules for strategic obstacle avoidance:
        # - Red obstacles (0): Report only if in center (1) or right (2) regions
        # - Green obstacles (1): Report only if in left (0) or center (1) regions
        # This prevents unnecessary avoidance maneuvers for obstacles on opposite sides
        if (color_index == 0 and roi > 0) or \
           (color_index == 1 and roi < 2):

            # Calculate obstacle section with color offset for UART transmission
            y_coord = closest_blob['rect'][1] + closest_blob['rect'][3]  # Bottom edge coordinate
            section = get_section_from_coords(closest_blob['cx'], y_coord, img.width())
            section_with_offset = section + (color_index * 6)  # Add color-based offset

            # Prepare UART message components
            blob_sect_str = str(section_with_offset)  # Convert section to string
            blob_roi_str = str(roi)                   # Convert ROI to string

            # Update LED indication to show detected obstacle color
            # Turn off default green LED to show specific obstacle color
            led_green.off()
            if color_index == 0:
                led_red.on()      # Red LED for red obstacle detection
            elif color_index == 1:
                led_green.on()    # Green LED for green obstacle detection (override default)

    # Generate final UART message with obstacle information
    mensaje_final = f"B,{blob_sect_str},{blob_roi_str}\r\n"

    # =================================================================
    # =========== UART COMMUNICATION AND STATUS INDICATION ==============
    # =================================================================
    # Send obstacle data to robot controller via UART protocol
    # LED status indication provides visual feedback about data transmission
    # Debug messages provide comprehensive system status information
    
    # Check if UART streaming is enabled (after successful CHECK completion)
    if streaming_habilitado:
        # Turn off blob detection LEDs to avoid interference with transmission status LEDs
        led_red.off()
        led_green.off()  # Blue LED already off
        led_blue.off()
        
        # Provide visual feedback based on message content
        if mensaje_final != "B,0,0\r\n":
            # White light (all LEDs) when sending non-zero obstacle data
            led_red.on()
            led_green.on()
            led_blue.on()
        else:
            # Purple light (red + blue) when sending "0,0" (no obstacle)
            led_red.on()
            led_blue.on()
            
        # Transmit obstacle data to robot controller via UART
        uart.write(mensaje_final)

        # Generate comprehensive debug message with system status
        debug_msg = f"St:{estado_camara[:4]} Avoided:{bloques_esquivados_actualmente}/{bloques_a_esquivar_max} Msg:{mensaje_final.strip()}"
    else:
        # Generate debug message for preview mode (streaming disabled)
        debug_msg = f"St:{estado_camara[:4]} Waiting for Cmd '2'... (Preview active) Msg:{mensaje_final.strip()}"

    # Output debug information for monitoring and troubleshooting
    print(debug_msg)

# =================================================================
# =========== MAIN PROCESSING LOOP COMPLETION ======================
# =================================================================
# Continuous vision processing loop provides real-time obstacle detection
# and navigation assistance for autonomous robot operation
# System maintains persistent tracking and adaptive communication protocols

```
