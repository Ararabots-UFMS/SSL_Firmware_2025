#include "robot_command.h"

RobotCommand decodeCommand(const char* buffer) {
    RobotCommand cmd;
    const uint8_t* bytes = (const uint8_t*)buffer;
    
    // Extract ID
    cmd.id = bytes[0] >> 4;
    
    // Extract and sign-extend vx_linear (20-bit value)
    uint32_t vx_raw = ((bytes[0] & 0x0F) << 16) | (bytes[1] << 8) | bytes[2];
    if (vx_raw & 0x80000) {  // Check if bit 19 is set (negative)
        vx_raw |= 0xFFF00000;  // Sign extend to 32 bits
    }
    cmd.vx_linear = (int32_t)vx_raw;
    
    // Extract and sign-extend vy_linear (20-bit value)
    uint32_t vy_raw = ((bytes[3] & 0xFF) << 12) | (bytes[4] << 4) | (bytes[5] >> 4);
    if (vy_raw & 0x80000) {  // Check if bit 19 is set (negative)
        vy_raw |= 0xFFF00000;  // Sign extend to 32 bits
    }
    cmd.vy_linear = (int32_t)vy_raw;
    
    // Extract and sign-extend angular_speed (20-bit value)
    uint32_t angular_raw = ((bytes[5] & 0x0F) << 16) | (bytes[6] << 8) | bytes[7];
    if (angular_raw & 0x80000) {  // Check if bit 19 is set (negative)
        angular_raw |= 0xFFF00000;  // Sign extend to 32 bits
    }
    cmd.angular_speed = (int32_t)angular_raw;
    
    // Extract and sign-extend angle (20-bit value)
    uint32_t angle_raw = ((bytes[8] & 0xFF) << 12) | (bytes[9] << 4) | (bytes[10] >> 4);
    if (angle_raw & 0x80000) {  // Check if bit 19 is set (negative)
        angle_raw |= 0xFFF00000;  // Sign extend to 32 bits
    }
    cmd.angle = (int32_t)angle_raw;
    
    // Extract kick_front flag
    cmd.kick_front = (bytes[10] & 0x08) != 0;
    
    return cmd;
}

void printCommand(const RobotCommand& cmd) {
    Serial.print("ID: "); Serial.println(cmd.id);
    Serial.print("VX Linear: "); Serial.println(cmd.vx_linear);
    Serial.print("VY Linear: "); Serial.println(cmd.vy_linear);
    Serial.print("Angular Speed: "); Serial.println(cmd.angular_speed);
    Serial.print("Angle: "); Serial.println(cmd.angle);
    Serial.print("Kick Front: "); Serial.println(cmd.kick_front ? "Yes" : "No");
}