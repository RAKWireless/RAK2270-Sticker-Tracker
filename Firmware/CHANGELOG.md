## [Unreleased] - 2025-04-10

### Added

### Changed

### Fixed

## [1.1.2] - 2025-04-10

Use RUI Arduino BSP RAK3172-SiP v4.1.1  

### Added
- add AT and Downlink command for re-join.  
- add AT and Downlink command for development testing.  

### Changed
- Optimize motion detection functionality.  
- Add stage management to optimize the re-join mechanism.  
- Configure the activation GPIO pin as pull-none, because add a pull-down resistor in the hardware to prevent excessive power consumption caused by software misjudgment.  
- Cancel the downlink command ACK to avoid conflicts caused by the RUI mechanism of automatically replying with an null packet.   

### Fixed
- Add a debounce mechanism to prevent accidental device activation during production.  
- Optimize the linkcheck mechanism to avoid conflicts caused by multiple timers.  

## [1.1.1] - 2024-12-20

Use RUI Arduino BSP RAK3172-SiP v4.1.1  

### Added
- support Motion Detection by the accelerometer.  
- add AT and Downlink command for Motion Detection.  

### Changed

### Fixed


## [1.1.0] - 2024-02-22

Use RUI Arduino BSP RAK3172-SiP v4.1.0  

### Added
- first release for RAK2270_1.1.0_Generic.  

### Changed

### Fixed

