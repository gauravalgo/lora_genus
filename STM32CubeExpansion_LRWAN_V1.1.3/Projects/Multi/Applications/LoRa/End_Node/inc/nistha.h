#include<stdint.h>
union Index_t
{
	struct day_t
	{
	uint8_t day : 6 ;
	uint8_t half_day : 2;
	}day_t;
	uint8_t index_t;
};
struct dummy
{
	uint8_t SOP;
	uint8_t Length;
};
struct Data_Downloaded_t
{
	uint8_t Meter_Vendor_Code;
	uint16_t Nistha_Version;
	uint32_t Serial_Number;
	uint8_t Meter_Make;
	uint8_t Meter_Type;
	uint32_t Meter_Time_Stamp;
	uint32_t Cumulative_Active_Energy;
	uint32_t Cumulative_Apparent_Energy;
	uint16_t Instantaneous_Voltage;
	uint16_t Instantaneous_Current;
	uint16_t Instantaneous_Load;
	uint32_t Tamper_Present_Status_t;
};
struct Command_Format
{
	uint8_t SOP;
	uint8_t Length;
	uint8_t Command_Type;
	uint8_t Sub_Command_Type;
	union Index_t Index;
	uint8_t Checksum;
};

struct Command_Response_Format
{
	uint8_t SOP;
	uint8_t Length;
	uint8_t Command_Type;
	uint8_t Sub_Command_Type;
	struct Data_Downloaded_t Data;
	uint8_t Checksum;
};
struct Data_Downloaded_BH
{
	uint32_t Cumulative_Active_Energy;
	uint32_t Cumulative_Apparent_Energy;
	uint16_t Maximum_Demand;
	uint32_t Maximum_Demand_Time_Stamp;
};
struct Billing_History_Response_Format
{
	uint8_t SOP;
	uint8_t Length;
	uint8_t Command_Type;
	uint8_t Sub_Command_Type;
	struct Data_Downloaded_BH Data;
	uint8_t Checksum;
};
struct Data_Downloaded_TD
{
	uint32_t Start_Time_Stamp_Latest_Event_1;
	uint32_t Stop_Time_Stamp_Latest_Event_1;
	uint32_t Start_Time_Stamp_Latest_Event_2;
	uint32_t Stop_Time_Stamp_Latest_Event_2;
	uint32_t Start_Time_Stamp_Latest_Event_3;
	uint32_t Stop_Time_Stamp_Latest_Event_3;
	uint32_t Start_Time_Stamp_Latest_Event_4;
	uint32_t Stop_Time_Stamp_Latest_Event_4;
	uint32_t Start_Time_Stamp_Latest_Event_5;
	uint32_t Stop_Time_Stamp_Latest_Event_5;
	uint32_t Start_Time_Stamp_Latest_Event_6;
	uint32_t Stop_Time_Stamp_Latest_Event_6;
};
struct Tamper_Data_Response_Format
{
	uint8_t SOP;
	uint8_t Length;
	uint8_t Command_Type;
	uint8_t Sub_Command_Type;
	struct Data_Downloaded_TD Data;
	uint8_t Checksum;
};
struct Data_Downloaded_MCOT
{
uint32_t Time_Stamp_Event;
};
struct TDRF_Meter_Cover_open_Tamper 
{
	uint8_t SOP;
	uint8_t Length;
	uint8_t Command_Type;
	uint8_t Sub_Command_Type;
	struct Data_Downloaded_MCOT Data;
	uint8_t Checksum;
};
struct Active_Energy_t
{
	uint8_t  hour_1;
	uint8_t  hour_1_end;
	uint8_t  hour_2;
	uint8_t  hour_2_end;
	uint8_t  hour_3;
	uint8_t  hour_3_end;
	uint8_t  hour_4;
	uint8_t  hour_4_end;
	uint8_t  hour_5;
	uint8_t  hour_5_end;
	uint8_t  hour_6;
	uint8_t  hour_6_end;
	uint8_t  hour_7;
	uint8_t  hour_7_end;
	uint8_t  hour_8;
	uint8_t  hour_8_end;
	uint8_t  hour_9;
	uint8_t  hour_9_end;
	uint8_t  hour_10;
	uint8_t  hour_10_end;
	uint8_t  hour_11;
	uint8_t  hour_11_end;
	uint8_t  hour_12;
	uint8_t  hour_12_end;
	
};
struct Average_Voltage_t
{
	uint16_t  hour_1_;
	uint16_t  hour_1_end_;
	uint16_t  hour_2_;
	uint16_t  hour_2_end_;
	uint16_t  hour_3_;
	uint16_t  hour_3_end_;
	uint16_t  hour_4_;
	uint16_t  hour_4_end_;
	uint16_t  hour_5_;
	uint16_t  hour_5_end_;
	uint16_t  hour_6_;
	uint16_t  hour_6_end_;
	uint16_t  hour_7_;
	uint16_t  hour_7_end_;
	uint16_t  hour_8_;
	uint16_t  hour_8_end_;
	uint16_t  hour_9_;
	uint16_t  hour_9_end_;
	uint16_t  hour_10_;
	uint16_t  hour_10_end_;
	uint16_t  hour_11_;
	uint16_t  hour_11_end_;
	uint16_t  hour_12_;
	uint16_t  hour_12_end_;
	
};
struct Data_Downloaded_LSDR_t
{
	uint8_t Index ;
	uint32_t UTC_Time;
	struct Active_Energy_t AE;
	struct Average_Voltage_t AV;
	
};
struct Load_Survey_Data_Response_Format
{
	uint8_t SOP;
	uint8_t Length;
	uint8_t Command_Type;
	uint8_t Sub_Command_Type;
	struct Data_Downloaded_LSDR_t Data;
	uint8_t Checksum;
};
/*struct Command_Format Billing_Data_Command_Format;
struct Command_Response_Format Billing_Response_Format;
struct Command_Format Billing_History_Command_Format;
struct Billing_History_Response_Format BHRF;
struct Command_Format Tamper_Data_Command_Format;
struct Tamper_Data_Response_Format  TDRF;
struct Command_Format Command_Format_Reversal_Phase_Neutral_Line_Load;
struct Tamper_Data_Response_Format  Reversal_Phase_Neutral_Line_Load_TDRF;
struct Command_Format Command_Format_Load_Earth_Tamper;
struct Tamper_Data_Response_Format  Load_Earth_Tamper_TDRF;
struct Command_Format Command_Format_Neutral_Disconnected;
struct Tamper_Data_Response_Format  Neutral_Disconnected_TDRF;
struct Command_Format Command_Format_Magnetic_Tamper;
struct Tamper_Data_Response_Format  Magnetic_Tamper_TDRF;
struct Command_Format Command_Format_Meter_Cover_open_Tamper;
struct TDRF_Meter_Cover_open_Tamper  Meter_Cover_open_Tamper_TDRF;
struct Command_Format Command_Format_Load_Survey_Data;
struct Load_Survey_Data_Response_Format LSDRF;*/


	




