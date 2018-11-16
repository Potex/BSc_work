#include <iostream>
#include <memory>
#include <unordered_set>

#include <boost/exception/diagnostic_information.hpp>
#include <class_loader/class_loader.hpp>
#include <socketcan_interface/socketcan.h>

using namespace can;

#include <iostream>

int rtflag=0;
int statecommand=0;
int act_state=0;
int auto_mode_active=0;
int zero =0;
int throttle_pos=0;
int act_gear=0;
int temp;

float veh_speed=0.0;
float wheel_ang=0.0;
float veh_spd_ref=0.0;
float serv_comm=0.0;
float serv_sens=0.0;
float steer_ang=0.0;
float nissan_veh_speed=0.0;
float WSpd_FR=0.0;
float WSpd_FL=0.0;
float WSpd_RR=0.0;
float WSpd_RL=0.0;

void print_error(const State & s);

void print_frame(const Frame &f){
	
    switch((int) f.id){
		case 0x2 :
			temp = f.data[0];
			steer_ang = temp*256;
			temp = f.data[1];
			steer_ang += temp;
			steer_ang = steer_ang/10/4684.8;
			std::cout << "Steering angle = " << steer_ang << "°" << std::endl;
			break;
			
		case 0x11 :
			rtflag = f.data[0];
			statecommand = f.data[1];
			std::cout << "RT flag = " << "\t" << rtflag << std::endl;
			std::cout << "State command = " << "\t" << statecommand << std::endl;
			break;
			
		case 0x12 :
			act_state = f.data[1];
			std::cout << "Actual state = " << "\t" << act_state << std::endl;
			break;
			
		case 0x176 :
			temp = f.data[0];
			nissan_veh_speed = temp*256;
			temp = f.data[1];
			nissan_veh_speed += temp;
			nissan_veh_speed = nissan_veh_speed/8.570637;
			std::cout << "Nissan Veh. speed = " << nissan_veh_speed << " km/h" << std::endl;
			break;
			
		case 0x180 :
			
			throttle_pos =  0,5 * (f.data[5]);
			std::cout << "Throttle pos = " << "\t" << throttle_pos << " %" <<  std::endl;
			break;
			
		case 0x284 :
			temp = f.data[0];
			WSpd_FR = temp*16777216;
			//std::cout << serv_comm << std::endl;
			temp = f.data[1];
			WSpd_FR += (temp*65536);
			temp = f.data[2];
			WSpd_FR += (temp*256);
			temp = f.data[3];
			WSpd_FR += temp;			
			temp = f.data[4];
			WSpd_FL = temp*16777216;
			temp = f.data[5];
			WSpd_FL += (temp*65536);
			temp = f.data[6];
			WSpd_FL += (temp*256);
			temp = f.data[7];
			WSpd_FL += temp;
			std::cout << "WSpeed Front-Right = " << "\t" << WSpd_FR << std::endl;
			std::cout << "WSpeed Front-Left = " << "\t" << WSpd_FL << std::endl;
			break;
			
		case 0x285 :
			temp = f.data[0];
			WSpd_RR = temp*16777216;
			//std::cout << serv_comm << std::endl;
			temp = f.data[1];
			WSpd_RR += (temp*65536);
			temp = f.data[2];
			WSpd_RR += (temp*256);
			temp = f.data[3];
			WSpd_RR += temp;			
			temp = f.data[4];
			WSpd_RL = temp*16777216;
			temp = f.data[5];
			WSpd_RL += (temp*65536);
			temp = f.data[6];
			WSpd_RL += (temp*256);
			temp = f.data[7];
			WSpd_RL += temp;
			std::cout << "WSpeed Rear-Right = " << "\t" << WSpd_RR << std::endl;
			std::cout << "WSpeed Rear-Left = " << "\t" << WSpd_RL << std::endl;
			break;
					
		case 0x421 :
			act_gear = f.data[0];
			std::cout << "Actual Gear = " << "\t" << act_gear << std::endl;
			break;
			
		default:
			//std::cout << "Not defined CAN message recieved!" << std::endl;
			;
		}
    
}

std::shared_ptr<class_loader::ClassLoader> g_loader;
DriverInterfaceSharedPtr g_driver;

void print_error(const State & s){
    std::string err;
    g_driver->translateError(s.internal_error,err);
    std::cout << "ERROR: state=" << s.driver_state << " internal_error=" << s.internal_error << "('" << err << "') asio: " << s.error_code << std::endl;
}

int main(int argc, char *argv[]){

    if(argc != 2 && argc != 4){
        std::cout << "usage: "<< argv[0] << " DEVICE [PLUGIN_PATH PLUGIN_NAME]" << std::endl;
        return 1;
    }

    if(argc == 4 ){
        try
        {
            g_loader = std::make_shared<class_loader::ClassLoader>(argv[2]);
            g_driver = g_loader->createUniqueInstance<DriverInterface>(argv[3]);
        }

        catch(std::exception& ex)
        {
            std::cerr << boost::diagnostic_information(ex) << std::endl;;
            return 1;
        }
    }else{
        g_driver = std::make_shared<SocketCANInterface>();
    }

    FrameListenerConstSharedPtr frame_printer = g_driver->createMsgListener(print_frame);
    StateListenerConstSharedPtr error_printer = g_driver->createStateListener(print_error);

    if(!g_driver->init(argv[1], false)){
        print_error(g_driver->getState());
        return 1;
    }

    g_driver->run();

    g_driver->shutdown();
    g_driver.reset();
    g_loader.reset();

    return 0;
}
