#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <controller_interface/controller_interface.hpp>

int main(int argc, char** argv)
{
  std::cout << "Verifying hsr_velocity_controller plugin registration..." << std::endl;
  std::cout << "============================================================" << std::endl;
  
  try {
    // Create a class loader for controller_interface plugins
    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
    
    // Get all available classes
    std::vector<std::string> classes = loader.getDeclaredClasses();
    
    std::cout << "\nFound " << classes.size() << " controller plugins:" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    
    std::string target = "hsr_velocity_controller_ns/HsrVelocityController";
    bool found = false;
    
    for (const auto& cls : classes) {
      if (cls == target) {
        std::cout << "✓ " << cls << " (TARGET - FOUND!)" << std::endl;
        found = true;
      } else {
        std::cout << "  " << cls << std::endl;
      }
    }
    
    std::cout << "\n============================================================" << std::endl;
    if (found) {
      std::cout << "✓ SUCCESS: Plugin 'hsr_velocity_controller_ns/HsrVelocityController'" << std::endl;
      std::cout << "  is properly registered and discoverable by pluginlib!" << std::endl;
      
      // Try to actually load it
      std::cout << "\nAttempting to instantiate the plugin..." << std::endl;
      auto instance = loader.createSharedInstance(target);
      if (instance) {
        std::cout << "✓ Plugin instantiation successful!" << std::endl;
      }
      return 0;
    } else {
      std::cout << "✗ ERROR: Plugin 'hsr_velocity_controller_ns/HsrVelocityController'" << std::endl;
      std::cout << "  was NOT found in the plugin registry!" << std::endl;
      return 1;
    }
    
  } catch (const std::exception& e) {
    std::cerr << "✗ ERROR: " << e.what() << std::endl;
    return 1;
  }
}
