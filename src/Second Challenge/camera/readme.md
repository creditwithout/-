# Open MV H7 PLUS SRC & Explanation

![Add a subheading (16 x 5 cm) (4096 x 2160 px) (11)](https://github.com/user-attachments/assets/06312ff0-18c5-40dc-a517-d2fe19bb8648)


### We'll explain the most important tecnologies software in the code, here you can see the [arduino Code](https://github.com/creditwithout/-/tree/main/src/Second%20Challenge) .

## LAB Color Space Thresholds Implementation

```py
# LAB color space thresholds for robust color detection under varying lighting
threshold_color_1 = (0, 100, 4, 127, -124, 127)    # Red/Orange obstacles
threshold_color_2 = (0, 100, -128, -18, 11, 127)   # Green obstacles  
threshold_color_3 = (0, 100, 8, 127, -128, 8)      # Blue lane markers

# LAB threshold application in blob detection
blobs_red = img.find_blobs([threshold_color_1], pixels_threshold=1000, merge=True)
for blob in blobs_red:
    if blob.density() > 0.8:  # Apply density filtering
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
```
The LAB color space provides superior color consistency compared to RGB under varying illumination conditions. LAB separates luminance (L) from chromaticity (A, B channels), making it ideal for robotic vision systems operating in changing light environments. The L channel represents lightness (0-100), while A channel ranges from green (-128) to red (+127), and B channel from blue (-128) to yellow (+127). Threshold format `[L_min, L_max, A_min, A_max, B_min, B_max]` allows precise color range definition. 

- Red obstacles use positive A values (4-127) indicating red dominance, while green obstacles use negative A values (-128 to -18) indicating green dominance. Blue lane markers utilize negative B values `(-128 to 8)` for blue detection. This LAB-based approach ensures reliable obstacle detection across different lighting scenarios, critical for autonomous navigation systems requiring consistent performance.

## Object Color Tracking with Simplified Kalman Filter

```py
# STEP 1: Velocity-based position prediction
for tb in tracked_blobs:
    tb['cx'] += tb['vx']  # Predict new X coordinate based on X velocity
    tb['cy'] += tb['vy']  # Predict new Y coordinate based on Y velocity

# STEP 2: Nearest neighbor association with distance calculation
for tracked_blob in tracked_blobs:
    min_dist_sq = MATCHING_DISTANCE_THRESHOLD**2
    for detection in all_detections_this_frame:
        if tracked_blob['color_index'] == detection['color_index']:
            dist_sq = (tracked_blob['cx'] - detection['blob'].cx())**2 + (tracked_blob['cy'] - detection['blob'].cy())**2
            if dist_sq < min_dist_sq:
                best_match = detection

# STEP 3: Simplified Kalman filter velocity correction
tracked_blob['vx'] = int((tracked_blob['vx'] * 0.5) + ((new_blob.cx() - tracked_blob['cx']) * 0.5))
tracked_blob['vy'] = int((tracked_blob['vy'] * 0.5) + ((new_blob.cy() - tracked_blob['cy']) * 0.5))
```

- The multi-object tracking system implements a simplified Kalman filter approach for robust obstacle tracking across video frames. The prediction step uses stored velocity vectors (vx, vy) to estimate blob positions in the next frame, improving association accuracy during temporary occlusions. The association phase employs nearest neighbor matching based on Euclidean distance between predicted positions and actual detections, ensuring consistent blob identity across frames. Color consistency is maintained by matching only blobs of identical color indices (red=0, green=1, blue=2), preventing false associations between different obstacle types. The Kalman correction step applies a weighted average (α=0.5) between previous velocity estimates and measured position differences, providing smooth velocity updates while filtering measurement noise. This simplified approach reduces computational complexity compared to full Kalman implementation while maintaining tracking robustness essential for autonomous navigation systems requiring reliable obstacle monitoring.

# End Page
Seteki 2025 - src -second challenge- camera
