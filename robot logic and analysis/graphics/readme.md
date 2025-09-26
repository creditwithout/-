
![Add a subheading (16 x 5 cm) (4096 x 2160 px) (4)](https://github.com/user-attachments/assets/72b55f8b-9091-4b58-a246-8826f7c33bf9)

# 1
<img width="1400" height="800" alt="Generated Image September 26, 2025 - 8_24AM" src="https://github.com/user-attachments/assets/de6ad322-db04-4097-b434-882e1a346bd5" />

- Ths `graph` illustrates the fundamental process of sensor fusion, specifically using a complementary filter to obtain an accurate and stable angle estimation. It masterfully showcases the individual weaknesses of an accelerometer and a gyroscope and demonstrates how combining them yields a superior result.

- The thin, erratic green line represents the angle derived from the `accelerometer`. While this signal is incredibly noisy and unreliable for instantaneous measurements due to its high sensitivity to external vibrations, its long-term average is highly accurate. It does not drift because it uses the constant force of gravity as its reference, correctly centering around the true angle of zero.
  
- In contrast, the smooth, thin red line shows the angle from the gyroscope. This signal is excellent for tracking rapid, short-term changes in orientation without noise. However, it suffers from a critical flaw known as "drift," where small, inherent errors accumulate over time, causing the angle to slowly but steadily wander away from the true value.
  
The thick blue line is the final, fused output from the complementary filter. This algorithm intelligently combines the best attributes of both sensors. It primarily relies on the smooth, responsive data from the gyroscope for its moment-to-moment readings. Simultaneously, it uses the noisy but stable long-term data from the accelerometer as a constant, reliable anchor.
This process effectively corrects the gyroscope's drift by continuously "pulling" its smooth signal back towards the correct average provided by the accelerometer.

# 2
![WhatsApp Image 2025-09-26 at 8 27 46 AM](https://github.com/user-attachments/assets/306d4c11-6e32-47e0-ab34-9debfb363ac7)

# 3
<img width="1000" height="600" alt="Generated Image September 26, 2025 - 8_08AM (1)" src="https://github.com/user-attachments/assets/e896548e-b0d2-492a-8add-6beeea507c53" />

# 4
<img width="1200" height="700" alt="Generated Image September 24, 2025 - 9_00PM (1)" src="https://github.com/user-attachments/assets/b6644634-17e4-4852-b234-5fa3efcad33c" />
