// Running Pikachu skeleton

module RunningPikachu
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        KEY,
        SW,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
        HEX0,
		HEX1,
		HEX2,
		HEX3,
		HEX4,
		HEX5
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;

	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	output  [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

	wire resetn, go;
	wire jump, squat;
	assign resetn = KEY[0];
	assign go = ~KEY[1];
	assign jump = ~KEY[2];
	assign squat = ~KEY[3];

	// lots of wires here

    wire draw_ground, finish_ground;
	wire draw_pikachu, finish_pikachu, en_erase;
	wire draw_pikachu_up, draw_pikachu_down;
	wire draw_cloud, finish_cloud, move;
	wire draw_bird, finish_bird;
	wire draw_tree, finish_tree;
	wire draw_death, finish_death, is_over;
	wire draw_new_death, move_death, draw_holding_bar;
	wire finish_erase, change_status;
	wire stop, is_jump, down;
	wire en_delay, delay_reset, control_reset;
	wire [4:0] holding, holding_out;
    wire finish_start_scene, finish_p, finish_i, finish_k, 
		 finish_a, finish_c, finish_h, finish_u, finish_r, 
		 finish_n, finish_g, finish_s, finish_y, finish_t;
	wire draw_start_scene, draw_p, draw_i, draw_k, draw_a,
		 draw_c, draw_h, draw_u, draw_r, draw_n, draw_g,
		 draw_s, draw_t;
    wire start_en_delay, finish_change_letter;
	wire draw_i1, draw_u1, draw_n1, draw_n2, draw_t1, draw_r1, draw_a1;
	wire draw_ground_low, finish_ground_low;
    wire draw_ball;
    wire start_e, erase_en_delay, finish_erase_start, en_erase_start;
    wire [23:0] count;
    wire is_count, outside_resetn;

    assign outside_resetn = (KEY[1] || resetn);
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
    wire [7:0] x;
    wire [6:0] y;
	wire writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    
    // Instansiate datapath
	// datapath d0(...);
	datapath d0(
        .clock(CLOCK_50),
        .resetn(resetn),

        .draw_ground(draw_ground),
		.draw_pikachu(draw_pikachu),
		.draw_pikachu_up(draw_pikachu_up),
		.draw_pikachu_down(draw_pikachu_down),
		.draw_death(draw_death),
		.draw_new_death(draw_new_death),
		.draw_holding_bar(draw_holding_bar),
		.draw_cloud(draw_cloud),
		.draw_bird(draw_bird),
		.draw_tree(draw_tree),
		.en_delay(en_delay),
		.delay_reset(delay_reset),
		.en_erase(en_erase),
		.move(move),
		.move_death(move_death),
		.is_jump(is_jump),
		.down(down),
		.control_reset(control_reset),
		.change_status(change_status),
        .draw_start_scene(draw_start_scene), 
		.draw_p(draw_p), 
		.draw_i(draw_i), 
		.draw_k(draw_k),
		.draw_a(draw_a),
		.draw_c(draw_c),
		.draw_h(draw_h),
		.draw_u(draw_u),
		.draw_r(draw_r),
		.draw_n(draw_n),
		.draw_g(draw_g), 
		.draw_s(draw_s),
		.draw_t(draw_t),
		.start_en_delay(start_en_delay),
        .draw_i1(draw_i1),
		.draw_u1(draw_u1),
		.draw_n1(draw_n1),
		.draw_n2(draw_n2),
		.draw_t1(draw_t1),
		.draw_r1(draw_r1),
		.draw_a1(draw_a1),
		.draw_ground_low(draw_ground_low),
		.draw_ball(draw_ball),
		.erase_en_delay(erase_en_delay),
		.en_erase_start(en_erase_start),

		.holding(holding),
        .x_out(x), 
        .y_out(y),
        .colour_out(colour),
        .finish_ground(finish_ground),
		.finish_pikachu(finish_pikachu),
		.finish_death(finish_death),
		.finish_erase(finish_erase),
		.finish_cloud(finish_cloud),
		.finish_bird(finish_bird),
		.finish_tree(finish_tree),
		.stop(stop),
		.is_over(is_over),
        .finish_start_scene(finish_start_scene), 
		.finish_p(finish_p), 
		.finish_i(finish_i), 
		.finish_k(finish_k),
		.finish_a(finish_a),
		.finish_c(finish_c),
		.finish_h(finish_h),
		.finish_u(finish_u),
		.finish_r(finish_r),
		.finish_n(finish_n),
		.finish_g(finish_g), 
		.finish_s(finish_s),
		.finish_t(finish_t),
		.finish_change_letter(finish_change_letter),
		.finish_ground_low(finish_ground_low),
		.start_e(start_e),
		.finish_erase_start(finish_erase_start),
        .is_count(is_count)
        );

    // Instansiate FSM control
    // control c0(...);
	control c0(
        .clock(CLOCK_50),
        .resetn(resetn),
        .go(go),

        .finish_ground(finish_ground),
		.finish_pikachu(finish_pikachu),
		.finish_death(finish_death),
		.finish_erase(finish_erase),
		.finish_cloud(finish_cloud),
		.finish_bird(finish_bird),
		.finish_tree(finish_tree),
		.stop(stop),
		.jump(jump),
		.squat(squat),
		.is_over(is_over),
		.y(y),
		.holding_out(holding_out),
        .finish_start_scene(finish_start_scene), 
		.finish_p(finish_p), 
		.finish_i(finish_i), 
		.finish_k(finish_k),
		.finish_a(finish_a),
		.finish_c(finish_c),
		.finish_h(finish_h),
		.finish_u(finish_u),
		.finish_r(finish_r),
		.finish_n(finish_n),
		.finish_g(finish_g), 
		.finish_s(finish_s),
		.finish_t(finish_t),
		.finish_change_letter(finish_change_letter),
		.finish_ground_low(finish_ground_low),
		.start_e(start_e),
		.finish_erase_start(finish_erase_start),

        .draw_start_scene(draw_start_scene), 
		.draw_p(draw_p), 
		.draw_i(draw_i), 
		.draw_k(draw_k),
		.draw_a(draw_a),
		.draw_c(draw_c),
		.draw_h(draw_h),
		.draw_u(draw_u),
		.draw_r(draw_r),
		.draw_n(draw_n),
		.draw_g(draw_g), 
		.draw_s(draw_s),
		.draw_t(draw_t),
		.start_en_delay(start_en_delay),
        .draw_i1(draw_i1),
		.draw_u1(draw_u1),
		.draw_n1(draw_n1),
		.draw_n2(draw_n2),
		.draw_t1(draw_t1),
		.draw_r1(draw_r1),
		.draw_a1(draw_a1),
		.draw_ground_low(draw_ground_low),
		.draw_ball(draw_ball),
		.erase_en_delay(erase_en_delay),
		.en_erase_start(en_erase_start),
        .draw_ground(draw_ground),
		.draw_pikachu(draw_pikachu),
		.draw_pikachu_up(draw_pikachu_up),
		.draw_pikachu_down(draw_pikachu_down),
		.draw_death(draw_death),
		.draw_new_death(draw_new_death),
		.draw_holding_bar(draw_holding_bar),
		.draw_cloud(draw_cloud),
		.draw_bird(draw_bird),
		.draw_tree(draw_tree),
		.en_delay(en_delay),
		.delay_reset(delay_reset),
		.en_erase(en_erase),
		.move(move),
		.move_death(move_death),
		.control_reset(control_reset),
		.is_jump(is_jump),
		.down(down),
		.change_status(change_status),
        .writeEn(writeEn)
        );
	
	//assign clear = (down && !change_status);

	ram32x5 r0(
		.address(5'd1),
		.clock(CLOCK_50),
		.data(holding),
		.wren((change_status || down)),
		.q(holding_out)
		);

    outsideCounter o1(
            .clk(CLOCK_50), 
            .resetn(KEY[1]), 
            .enable(is_count),
            .hex0(HEX0), 
            .hex1(HEX1), 
            .hex2(HEX2), 
            .hex3(HEX3), 
            .hex4(HEX4), 
            .hex5(HEX5)
            );
endmodule

module control(

input clock,
input resetn,
input go,

input finish_ground,
input finish_pikachu,
input finish_death,
input finish_erase,
input finish_cloud,
input finish_bird,
input finish_tree,
input stop,
input jump,
input squat,
input is_over,
input [6:0] y,
input [4:0] holding_out,
input finish_start_scene, 
input finish_p, 
input finish_i, 
input finish_k,
input finish_a,
input finish_c,
input finish_h,
input finish_u,
input finish_r,
input finish_n,
input finish_g, 
input finish_s,
input finish_t,
input finish_change_letter,
input finish_ground_low,
input start_e,
input finish_erase_start,

output reg draw_start_scene,
output reg draw_p, 
output reg draw_i, 
output reg draw_k,
output reg draw_a,
output reg draw_c,
output reg draw_h,
output reg draw_u,
output reg draw_r,
output reg draw_n,
output reg draw_g, 
output reg draw_s,
output reg draw_t,
output reg start_en_delay,
output reg draw_i1,
output reg draw_u1,
output reg draw_n1,
output reg draw_n2,
output reg draw_t1,
output reg draw_r1,
output reg draw_a1,
output reg draw_ground_low,
output reg draw_ball,
output reg erase_en_delay,
output reg en_erase_start,
output reg draw_ground,
output reg draw_pikachu,
output reg draw_pikachu_up,
output reg draw_pikachu_down,
output reg draw_death,
output reg draw_new_death,
output reg draw_holding_bar,
output reg draw_cloud,
output reg draw_bird,
output reg draw_tree,
output reg en_delay,
output reg delay_reset,
output reg en_erase,
output reg move,
output reg move_death,
output reg control_reset,
output reg is_jump,
output reg down,
output reg change_status,
output reg writeEn
);

	reg [5:0] current_state, next_state;

	localparam  INITIAL = 6'd0,
				DRAW_GROUND_LOW = 6'd1,
				DRAW_BALL = 6'd2,
				DRAW_START_SCENE = 6'd3,
				DRAW_P = 6'd4,
				DRAW_I = 6'd5,
				DRAW_K = 6'd6,
				DRAW_A = 6'd7,
				DRAW_C = 6'd8,
				DRAW_H = 6'd9,
				DRAW_U = 6'd10,
				DRAW_R = 6'd11,
				DRAW_N = 6'd12,
				DRAW_G = 6'd13,
				DRAW_S = 6'd14,
				DRAW_T = 6'd15,
				DRAW_A_1 = 6'd16,
				DRAW_R_1 = 6'd17,
				DRAW_T_1 = 6'd18,
				DRAW_N_1 = 6'd19,
				DRAW_N_2 = 6'd20,
				DRAW_I_1 = 6'd21,
				DRAW_U_1 = 6'd22,
				COUNT_FRAME_START = 6'd23,
				CHOOSE_LETTER = 6'd24,
				COUNT_FRAME_ERASE = 6'd25,
				ERASE_START = 6'd26,
                START      = 6'd27,
				START_WAIT = 6'd28,
				DRAW_GROUND   = 6'd29,
				RESET = 6'd30,
				DRAW_DEATH = 6'd31,
				DRAW_PIKACHU = 6'd32,
				DRAW_PIKACHU_DOWN = 6'd33,
				DRAW_HOLDING_BAR = 6'd34,
				DRAW_PIKACHU_UP = 6'd35,
				DRAW_CLOUD = 6'd36,
				DRAW_BIRD = 6'd37,
				DRAW_TREE = 6'd38,
				COUNT_FRAME = 6'd39,
				ERASE = 6'd40,
				MOVE = 6'd41,
				COUNT_FRAME_DEATH = 6'd42,
				ERASE_DEATH = 6'd43,
				MOVE_DEATH = 6'd44,
				RESET_DEATH = 6'd45,
				DRAW_NEW_DEATH = 6'd46;
	
	always @(*)
	begin : state_diagram
		case (current_state)
            INITIAL: next_state = DRAW_GROUND_LOW;
				DRAW_GROUND_LOW: next_state = finish_ground_low ? DRAW_BALL : DRAW_GROUND_LOW;
				DRAW_BALL: next_state = finish_death ? DRAW_START_SCENE: DRAW_BALL;
				DRAW_START_SCENE: next_state = finish_start_scene ? COUNT_FRAME_START : DRAW_START_SCENE;
				DRAW_P: next_state = finish_p ? COUNT_FRAME_START : DRAW_P;
				DRAW_I: next_state = finish_i ? COUNT_FRAME_START : DRAW_I;
				DRAW_K: next_state = finish_k ? COUNT_FRAME_START : DRAW_K;
				DRAW_A: next_state = finish_a ? COUNT_FRAME_START : DRAW_A;
				DRAW_C: next_state = finish_c ? COUNT_FRAME_START : DRAW_C;
				DRAW_H: next_state = finish_h ? COUNT_FRAME_START : DRAW_H;
				DRAW_U: next_state = finish_u ? COUNT_FRAME_START : DRAW_U;
				DRAW_R: next_state = finish_r ? COUNT_FRAME_START : DRAW_R;
				DRAW_U_1: next_state = finish_u ? COUNT_FRAME_START : DRAW_U_1;
				DRAW_N: next_state = finish_n ? COUNT_FRAME_START : DRAW_N;
				DRAW_N_1: next_state = finish_n ? COUNT_FRAME_START : DRAW_N_1;
				DRAW_I_1: next_state = finish_i ? COUNT_FRAME_START : DRAW_I_1;
				DRAW_N_2: next_state = finish_n ? COUNT_FRAME_START : DRAW_N_2;
				DRAW_G: next_state = finish_g ? COUNT_FRAME_START : DRAW_G;
				DRAW_S: next_state = finish_s ? DRAW_T : DRAW_S;
				DRAW_T: next_state = finish_t ? DRAW_A_1 : DRAW_T;
				DRAW_A_1: next_state = finish_a ? DRAW_R_1 : DRAW_A_1;
				DRAW_R_1: next_state = finish_r ? DRAW_T_1 : DRAW_R_1;
				DRAW_T_1: next_state = finish_t ? COUNT_FRAME_ERASE : DRAW_T_1;
				COUNT_FRAME_ERASE: if (go) begin
									next_state = START;
								end
								else if (start_e) begin
									next_state = change_path ? ERASE_START : DRAW_S;
								end
								else begin
									next_state = COUNT_FRAME_ERASE;
								end
				ERASE_START: next_state = finish_erase_start ? COUNT_FRAME_ERASE : ERASE_START;
				COUNT_FRAME_START: next_state = finish_change_letter ? CHOOSE_LETTER : COUNT_FRAME_START;
				CHOOSE_LETTER: begin
					if (change_time == 6'd0) begin
						next_state = DRAW_P;
					end
					else if (change_time == 6'd1) begin
						next_state = DRAW_I;
					end
					else if (change_time == 6'd2) begin
						next_state = DRAW_K;
					end
					else if (change_time == 6'd3) begin
						next_state = DRAW_A;
					end
					else if (change_time == 6'd4) begin
						next_state = DRAW_C;
					end
					else if (change_time == 6'd5) begin
						next_state = DRAW_H;
					end
					else if (change_time == 6'd6) begin
						next_state = DRAW_U;
					end
					else if (change_time == 6'd7) begin
						next_state = DRAW_R;
					end
					else if (change_time == 6'd8) begin
						next_state = DRAW_U_1;
					end
					else if (change_time == 6'd9) begin
						next_state = DRAW_N;
					end
					else if (change_time == 6'd10) begin
						next_state = DRAW_N_1;
					end
					else if (change_time == 6'd11) begin
						next_state = DRAW_I_1;
					end
					else if (change_time == 6'd12) begin
						next_state = DRAW_N_2;
					end
					else if (change_time == 6'd13) begin
						next_state = DRAW_G;
					end
					else if (change_time == 6'd14) begin
						next_state = DRAW_S;
					end
				end
			START: next_state = go ? START_WAIT : START;
			START_WAIT: next_state = go ? START_WAIT : DRAW_GROUND;
			DRAW_GROUND: next_state = finish_ground ? RESET : DRAW_GROUND;
			RESET: next_state = is_over ? DRAW_DEATH : DRAW_PIKACHU;
			DRAW_PIKACHU: next_state = change_status ? DRAW_PIKACHU_DOWN : DRAW_PIKACHU_UP;
			DRAW_DEATH: if (finish_death) begin
							next_state = change_status ? COUNT_FRAME : DRAW_NEW_DEATH;
						end
						else begin
							next_state = DRAW_DEATH;
						end//next_state = finish_death ? COUNT_FRAME : DRAW_DEATH;
			DRAW_PIKACHU_DOWN: next_state = finish_pikachu ? DRAW_HOLDING_BAR : DRAW_PIKACHU_DOWN;
			DRAW_PIKACHU_UP: next_state = finish_pikachu ? DRAW_CLOUD : DRAW_PIKACHU_UP;
			DRAW_HOLDING_BAR: next_state = DRAW_CLOUD;
			DRAW_CLOUD: next_state = finish_cloud ? DRAW_BIRD : DRAW_CLOUD;
			DRAW_BIRD: next_state = finish_bird ? DRAW_TREE : DRAW_BIRD;
			DRAW_TREE: next_state = finish_tree ? COUNT_FRAME : DRAW_TREE;
			COUNT_FRAME: if (stop) begin							
							if (is_over && change_status) begin
								next_state = INITIAL;
							end
							else begin
								next_state = ERASE;
							end
						end
						else begin
							next_state = COUNT_FRAME;
						end//next_state = stop ? ERASE : COUNT_FRAME;
			ERASE: next_state = finish_erase ? MOVE : ERASE;
			MOVE: next_state = RESET;

			RESET_DEATH: next_state = DRAW_NEW_DEATH;
			DRAW_NEW_DEATH: if (finish_death) begin
								if (y == 7'd115) begin
									next_state = INITIAL;
								end
								else begin
									next_state = COUNT_FRAME_DEATH;
								end
							end
							else begin
								next_state = DRAW_NEW_DEATH;
							end//finish_death ? COUNT_FRAME_DEATH : DRAW_NEW_DEATH;
			COUNT_FRAME_DEATH: next_state = stop ? ERASE_DEATH : COUNT_FRAME_DEATH;
			ERASE_DEATH: next_state = finish_erase ? MOVE_DEATH : ERASE_DEATH;
			MOVE_DEATH: next_state = RESET_DEATH;
			default: next_state = INITIAL;
		endcase
	end

	always @(*)
	begin
        draw_start_scene = 1'b0;
		draw_p = 1'b0;
		draw_k = 1'b0;
		draw_a = 1'b0;
		draw_c = 1'b0;
		draw_h = 1'b0;
		draw_u = 1'b0;
		draw_r = 1'b0;
		draw_g = 1'b0;
		draw_i = 1'b0;
		draw_n = 1'b0;
		start_en_delay = 1'b0;
		draw_i1 = 1'b0;
		draw_u1 = 1'b0;
		draw_n1 = 1'b0;
		draw_n2 = 1'b0;
		draw_t1 = 1'b0;
		draw_r1 = 1'b0;
		draw_a1 = 1'b0;
		draw_s = 1'b0;
		draw_t = 1'b0;
        draw_ground_low = 1'b0;
		draw_ball = 1'b0;
		erase_en_delay = 1'b0;
		en_erase_start = 1'b0;
		draw_ground = 1'b0;
		draw_pikachu = 1'b0;
		draw_pikachu_up = 1'b0;
		draw_pikachu_down = 1'b0;
		draw_cloud = 1'b0;
		draw_bird = 1'b0;
		draw_tree = 1'b0;
		draw_death = 1'b0;
		draw_new_death = 1'b0;
		draw_holding_bar = 1'b0;
		en_change = 1'b0;
		en_delay = 1'b0;
		delay_reset = 1'b0;
		en_erase = 1'b0;
		move = 1'b0;
		move_death = 1'b0;
		control_reset = 1'b0;
		writeEn = 1'b0;

		case (current_state)
            INITIAL: begin
				control_reset = 1'b1;
				delay_reset = 1'b1;
			end
			DRAW_GROUND_LOW: begin
				draw_ground_low = 1'b1;
                writeEn = 1'b1;
			end
			DRAW_BALL: begin
				draw_ball = 1'b1;
                writeEn = 1'b1;
			end
			DRAW_START_SCENE: begin
				draw_start_scene = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_P: begin
				draw_p = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_I: begin
				draw_i = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_K: begin
				draw_k = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_A: begin
				draw_a = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_C: begin
				draw_c = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_H: begin
				draw_h = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_U: begin
				draw_u = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_R: begin
				draw_r = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_U_1: begin
				draw_u1 = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_N: begin
				draw_n = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_N_1: begin
				draw_n1 = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_I_1: begin
				draw_i1 = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_N_2: begin
				draw_n2 = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_G: begin
				draw_g = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_S: begin
				draw_s = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_T: begin
				draw_t = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_A_1: begin
				draw_a1 = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_R_1: begin
				draw_r1 = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_T_1: begin
				draw_t1 = 1'b1;
				writeEn = 1'b1;
			end
			COUNT_FRAME_ERASE: begin
				erase_en_delay = 1'b1;
			end
			ERASE_START: begin
				delay_reset = 1'b1;
				en_erase_start = 1'b1;
				writeEn = 1'b1;
			end
			COUNT_FRAME_START: begin
				start_en_delay = 1'b1;
			end			
			CHOOSE_LETTER: begin
				delay_reset = 1'b1;
			end
			DRAW_GROUND: begin
				control_reset = 1'b1;
                draw_ground = 1'b1;
                writeEn = 1'b1;
			end
			RESET: begin
				delay_reset = 1'b1;
			end
			DRAW_PIKACHU: begin
				en_change = 1'b1;
			end
			DRAW_DEATH: begin
				draw_death = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_PIKACHU_DOWN: begin
				draw_pikachu = 1'b1;
				draw_pikachu_down = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_HOLDING_BAR: begin
				draw_holding_bar = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_PIKACHU_UP: begin
				draw_pikachu = 1'b1;
				draw_pikachu_up = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_CLOUD: begin
				draw_cloud = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_BIRD: begin
				draw_bird = 1'b1;
				writeEn = 1'b1;
			end
			DRAW_TREE: begin
				draw_tree = 1'b1;
				writeEn = 1'b1;
			end
			COUNT_FRAME: begin
				en_delay = 1'b1;
			end
			ERASE: begin
				en_erase = 1'b1;
				writeEn = 1'b1;
			end
			MOVE: begin
				move = 1'b1;
			end

			RESET_DEATH: begin
				delay_reset = 1'b1;
			end
			DRAW_NEW_DEATH: begin
				draw_new_death = 1'b1;
				writeEn = 1'b1;
			end
			COUNT_FRAME_DEATH: begin
				en_delay = 1'b1;
			end
			ERASE_DEATH: begin
				en_erase = 1'b1;
				writeEn = 1'b1;
			end
			MOVE_DEATH: begin
				move_death = 1'b1;
			end
		endcase
	end

	always @(posedge clock) begin
		if (!resetn) current_state <= INITIAL;
		else begin
			current_state <= next_state;
		end
	end

    reg [5:0] change_time;
	always @(posedge clock) begin
		if (!resetn || control_reset) begin
			change_time <= 6'd0;
		end
		else if (current_state == CHOOSE_LETTER) begin
			change_time <= change_time + 1'b1;
		end
	end

	reg change_path;
	always @(posedge clock) begin
		if (!resetn || control_reset) begin
			change_path <= 1'b0;
		end
		else if (current_state == DRAW_S) begin
			change_path <= 1'b1;
		end
		else if (current_state == ERASE_START) begin
			change_path <= 1'b0;
		end
	end

	// jump
	always @(posedge clock) 
	begin
		if (!resetn || current_state == START) begin
			is_jump <= 1'b0;
		end
		else if (jump) begin
			is_jump <= 1'b1;
		end
		else if (down == 1'b1 && y == 7'd115 && current_state == DRAW_PIKACHU_UP) begin
			is_jump <= 1'b0;
		end
	end

	// change moving direction
	always @(posedge clock)
	begin
		if (!resetn || current_state == START) begin
			down <= 1'b0;
		end	
		else begin
			if (y == 7'd115 && current_state == DRAW_PIKACHU_UP) begin
				down <= 1'b0;
			end
			if ((y == 7'd90 - holding_out) && current_state == DRAW_PIKACHU_UP) begin
				down <= 1'b1;
			end
		end	
	end

	// change status signal
	reg en_change;
	always @(posedge clock)
	begin: change_status_register
		if (!resetn) begin
			change_status <= 1'b0;
		end	
		else if (en_change) begin
			change_status <= !is_jump && squat;
		end
	end
endmodule


module datapath(

input clock,
input resetn,
input draw_ground,
input draw_pikachu,
input draw_pikachu_up,
input draw_pikachu_down,
input draw_death,
input draw_new_death,
input draw_holding_bar,
input draw_cloud,
input draw_bird,
input draw_tree,
input en_delay,
input delay_reset,
input en_erase,
input move,
input move_death,
input is_jump,
input down,
input control_reset,
input change_status,
input draw_start_scene, 
input draw_p, 
input draw_i, 
input draw_k,
input draw_a,
input draw_c,
input draw_h,
input draw_u,
input draw_r,
input draw_n,
input draw_g, 
input draw_s,
input draw_t,
input start_en_delay,
input draw_i1,
input draw_u1,
input draw_n1,
input draw_n2,
input draw_t1,
input draw_r1,
input draw_a1,
input draw_ground_low,
input draw_ball,
input erase_en_delay,
input en_erase_start,

output reg [4:0] holding,
output reg [7:0] x_out, 
output reg [6:0] y_out,
output reg [2:0] colour_out,
output reg finish_ground,
output reg finish_pikachu,
output reg finish_death,
output reg finish_erase,
output reg finish_cloud,
output reg finish_bird,
output reg finish_tree,
output reg is_over,
output stop,
output reg finish_start_scene, 
output reg finish_p, 
output reg finish_i, 
output reg finish_k,
output reg finish_a,
output reg finish_c,
output reg finish_h,
output reg finish_u,
output reg finish_r,
output reg finish_n,
output reg finish_g, 
output reg finish_s,
output reg finish_t,
output finish_change_letter,
output reg finish_ground_low,
output start_e,
output reg finish_erase_start,
output reg is_count
);

    //OUTPUT counter
    always @(*)
    begin
        if(draw_ground)
			is_count = 1;
		if(is_over)
			is_count = 0;
    end
	// Game over change signal
	always @(posedge clock)
	begin : game_over_register
		if (!resetn || control_reset) begin
			is_over <= 1'b0;
		end
		else if (move) begin
			if (change_status) begin
				if (pikachu_move_x + 3'd6 >= tree_move_x && pikachu_move_x + 3'd6 <= tree_move_x + 3'd3) begin
					is_over <= 1'b1;
				end
				if (7'd111 >= bird_move_y && 7'd111 <= bird_move_y + 3'd5) begin
					if (pikachu_move_x + 4'd6 >= bird_move_y && pikachu_move_x + 4'd6 <= bird_move_y + 3'd5) begin
						is_over <= 1'b1;
					end
				end
			end
			else if (!change_status) begin
				if (is_jump) begin
					if ((pikachu_move_x + 3'd2 >= tree_move_x && pikachu_move_x + 3'd2 <= tree_move_x + 3'd3) || 
						(pikachu_move_x + 3'd4 >= tree_move_x && pikachu_move_x + 3'd4 <= tree_move_x + 3'd3)) begin
							if ((pikachu_move_y >= tree_move_y && pikachu_move_y <= tree_move_y + 4'd11) || 
								(pikachu_move_y + 4'd6 >= tree_move_y && pikachu_move_y + 4'd6 <= tree_move_y + 4'd11)) begin
									is_over <= 1'b1;
								end
						end
					if ((pikachu_move_x + 3'd2 >= bird_move_x && pikachu_move_x + 3'd2 <= bird_move_x + 3'd6) ||
						(pikachu_move_x + 3'd4 >= bird_move_x && pikachu_move_x + 3'd4 <= bird_move_x + 3'd6)) begin
							if ((pikachu_move_y >= bird_move_y && pikachu_move_y <= bird_move_y + 3'd5) ||
								(pikachu_move_y + 4'd6 >= bird_move_y && pikachu_move_y + 4'd6 <= bird_move_y + 3'd5)) begin
									is_over <= 1'b1;
								end
						end	
				end
				else if (!is_jump) begin
					if (pikachu_move_x + 3'd4 >= tree_move_x && pikachu_move_x + 3'd4 <= tree_move_x + 3'd3) begin
						is_over <= 1'b1;
					end
					if ((7'd108 >= bird_move_y && 7'd108 <= bird_move_y + 3'd5) || 
						(7'd111 >= bird_move_y && 7'd111 <= bird_move_y + 3'd5) || 
						(7'd115 >= bird_move_y && 7'd115 <= bird_move_y + 3'd5)) begin
						if (4'd13 >= bird_move_x && 4'd13 <= bird_move_x + 3'd6) begin
							is_over <= 1'b1;
						end
					end
				end
			end
		end
	end


    // Initial game start scene
	reg [7:0] game_start_x_counter;
	reg [6:0] game_start_y_counter;
	always @(posedge clock) 
	begin : game_start_scene
		if (!resetn || finish_change_letter) begin
			game_start_x_counter <= 8'd0;
            game_start_y_counter <= 7'd0;
			finish_start_scene <= 1'b0;
		end
		else if(draw_start_scene) begin
			if (game_start_x_counter == 8'd79) begin
				if (game_start_y_counter == 7'd31) begin
					finish_start_scene = 1'b1;
					game_start_y_counter <= 7'd0;
				end
				else begin
					finish_start_scene <= 1'b0;
					game_start_y_counter <= game_start_y_counter + 1'b1;
				end
				game_start_x_counter <= 8'd0;
			end
			else begin
				finish_start_scene <= 1'b0;
				game_start_x_counter <= game_start_x_counter + 1'b1;
			end
		end
		else if (draw_p || draw_k || draw_c ||
				 draw_h || draw_g || draw_s) begin
			if (count_letter_pixel >= 6'd0 && count_letter_pixel <= 6'd4) begin
				game_start_x_counter <= count_letter_pixel;
				game_start_y_counter <= 7'd0;
			end
			else if (count_letter_pixel >= 6'd5 && count_letter_pixel <= 6'd9) begin
				game_start_x_counter <= count_letter_pixel - 6'd5;
				game_start_y_counter <= 7'd1;
			end
			else if (count_letter_pixel >= 6'd10 && count_letter_pixel <= 6'd14) begin
				game_start_x_counter <= count_letter_pixel - 6'd10;
				game_start_y_counter <= 7'd2;
			end
			else if (count_letter_pixel >= 6'd15 && count_letter_pixel <= 6'd19) begin
				game_start_x_counter <= count_letter_pixel - 6'd15;
				game_start_y_counter <= 7'd3;
			end
			else if (count_letter_pixel >= 6'd20 && count_letter_pixel <= 6'd24) begin
				game_start_x_counter <= count_letter_pixel - 6'd20;
				game_start_y_counter <= 7'd4;
			end
			else if (count_letter_pixel >= 6'd25 && count_letter_pixel <= 6'd29) begin
				game_start_x_counter <= count_letter_pixel - 6'd25;
				game_start_y_counter <= 7'd5;
			end
			else if (count_letter_pixel >= 6'd30 && count_letter_pixel <= 6'd34) begin
				game_start_x_counter <= count_letter_pixel - 6'd30;
				game_start_y_counter <= 7'd6;
			end
		end
	end
	reg [2:0] game_start_colour;
	// The colour change for the game_start_colour
    always @(*) 
    begin : game_start_colour_change
        if (!resetn) begin
            game_start_colour = 3'b000;
        end
        else if (draw_start_scene) begin
			if (game_start_y_counter == 7'd0) begin
				game_start_colour = 3'b100;
			end
			else if (game_start_y_counter >= 7'd1 && game_start_y_counter <= 7'd7) begin
				if (game_start_x_counter == 8'd0 || game_start_x_counter == 8'd79) begin
					game_start_colour = 3'b100;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else if (game_start_y_counter >= 7'd9 && game_start_y_counter <= 7'd15) begin
				if (game_start_x_counter == 8'd0 || game_start_x_counter == 8'd79) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else if (game_start_y_counter == 7'd16) begin
				game_start_colour = 3'b111;
			end
			else begin
				game_start_colour = 3'b000;
			end
        end
		else if (draw_p) begin
			if (game_start_y_counter == 7'd0 || game_start_y_counter == 7'd3) begin
				if (game_start_x_counter >= 8'd0 && game_start_x_counter <= 8'd2) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else if (game_start_y_counter == 7'd1 || game_start_y_counter == 7'd2) begin
				if (game_start_x_counter == 8'd0 || game_start_x_counter == 8'd3) begin
					game_start_colour = 3'b111;
				end
				else if (!(game_start_x_counter == 8'd0 || game_start_x_counter == 8'd3)) begin
					game_start_colour = 3'b000;
				end
			end 
			else if (game_start_y_counter >= 7'd4 && game_start_y_counter <= 7'd6)  begin 
				if (game_start_x_counter == 8'd0) begin
					game_start_colour = 3'b111;
				end
				else if (!(game_start_x_counter == 8'd0)) begin
					game_start_colour = 3'b000;
				end
			end
		end
		else if (draw_k) begin
			if (game_start_x_counter == 8'd0) begin
				game_start_colour = 3'b111;
			end
			else if (game_start_x_counter == 8'd2) begin
				if (game_start_y_counter == 8'd1 || game_start_y_counter == 8'd2 ||
					game_start_y_counter == 8'd4 || game_start_y_counter == 8'd5) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else if (game_start_y_counter == 8'd0 || game_start_y_counter == 8'd6) begin
				if (game_start_x_counter == 8'd0 || game_start_x_counter == 8'd3) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else if (game_start_y_counter == 8'd3) begin
				if (game_start_x_counter == 8'd0 || game_start_x_counter == 8'd1) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else begin
				game_start_colour = 3'b000;
			end
		end
		else if (draw_c) begin
			if (game_start_y_counter == 8'd0 || game_start_y_counter == 8'd6) begin
				if (game_start_x_counter == 8'd1 || game_start_x_counter == 8'd2) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else if (game_start_x_counter == 8'd0) begin
				if (game_start_y_counter == 8'd0 || game_start_y_counter == 8'd6) begin
					game_start_colour = 3'b000;
				end
				else begin
					game_start_colour = 3'b111;
				end
			end
			else if (game_start_x_counter == 8'd3) begin
				if (game_start_y_counter == 8'd1 || game_start_y_counter == 8'd5) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else begin
				game_start_colour = 3'b000;
			end
		end
		else if (draw_h) begin
			if (game_start_x_counter == 8'd0 || game_start_x_counter == 8'd3) begin
				game_start_colour = 3'b111;
			end
			else if (game_start_y_counter == 8'd3) begin
				if (game_start_x_counter >= 8'd0 && game_start_x_counter <= 8'd3) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else begin
				game_start_colour = 3'b000;
			end
		end
		else if (draw_g) begin
			if (game_start_y_counter == 7'd0 || game_start_y_counter == 7'd6) begin
				if (game_start_x_counter == 8'd1 || game_start_x_counter == 8'd2) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else if (game_start_x_counter == 8'd0) begin
				if (game_start_y_counter == 7'd0 || game_start_y_counter == 7'd6) begin
					game_start_colour = 3'b000;
				end
				else begin
					game_start_colour = 3'b111;
				end
			end
			else if (game_start_x_counter == 8'd3) begin
				if (game_start_y_counter == 7'd0 || game_start_y_counter == 7'd2 || game_start_y_counter == 7'd6) begin
					game_start_colour = 3'b000;
				end
				else begin
					game_start_colour = 3'b111;
				end
			end
			else if (game_start_y_counter == 7'd3) begin
				if (game_start_x_counter == 8'd0 || game_start_x_counter == 8'd2 || game_start_x_counter == 8'd3) begin
					game_start_colour = 3'b111;
				end
				else begin
					game_start_colour = 3'b000;
				end
			end
			else begin
				game_start_colour = 3'b000;
			end
		end 
		else if (draw_s) begin
			if (game_start_x_counter == 8'd0) begin
				if (game_start_y_counter == 7'd4 || game_start_y_counter == 7'd5) begin
					game_start_colour = 3'b000;
				end
				else begin
					game_start_colour = 3'b111;
				end
			end
			else if (game_start_x_counter == 8'd3) begin
				if (game_start_y_counter == 7'd1 || game_start_y_counter == 7'd2) begin
					game_start_colour = 3'b000;
				end
				else begin
					game_start_colour = 3'b111;
				end
			end
			else if (game_start_y_counter == 7'd0 || game_start_y_counter == 7'd3 || game_start_y_counter == 7'd6) begin
				if (game_start_x_counter == 8'd4) begin
					game_start_colour = 3'b000;
				end
				else begin
					game_start_colour = 3'b111;
				end
			end
			else begin
				game_start_colour = 3'b000;
			end
		end 
	end


    reg [7:0] i_x_counter;
	reg [6:0] i_y_counter;
	always @(posedge clock) 
	begin : i_xy
		if (!resetn || finish_change_letter) begin
			i_x_counter <= 8'd0;
            i_y_counter <= 7'd0;
			finish_i <= 1'b0;
		end
		else if (draw_i || draw_i1) begin
			if (i_x_counter == 8'd4) begin
				if (i_y_counter == 7'd6) begin
					finish_i <= 1'b1;
					i_y_counter <= 7'd0;
				end
				else begin
					finish_i <= 1'b0;
					i_y_counter <= i_y_counter + 1'b1;
				end
			i_x_counter <= 8'd0;
			end
			else begin
				finish_i <= 1'b0;
				i_x_counter <= i_x_counter + 1'b1;
			end
		end	
	end
	reg [2:0] i_colour;
	always @(*) 
    begin : i_colour_change
        if (!resetn) begin
            i_colour = 3'b000;
        end
		else if (draw_i) begin
			if (i_y_counter == 7'd0 || i_y_counter == 7'd6) begin
				if (i_x_counter >= 8'd0 && i_x_counter <= 8'd2) begin
					i_colour = 3'b111;
				end
				else begin
					i_colour = 3'b000;
				end
			end
			else if (i_x_counter == 8'd1) begin
				i_colour = 3'b111;
			end
			else begin
				i_colour = 3'b000;
			end
		end
	end
	reg [2:0] i1_colour;
	always @(*) 
    begin : i1_colour_change
        if (!resetn) begin
            i1_colour = 3'b000;
        end
		else if (draw_i1) begin
			if (i_y_counter == 7'd0 || i_y_counter == 7'd6) begin
				if (i_x_counter >= 8'd0 && i_x_counter <= 8'd2) begin
					i1_colour = 3'b111;
				end
				else begin
					i1_colour = 3'b000;
				end
			end
			else if (i_x_counter == 8'd1) begin
				i1_colour = 3'b111;
			end
			else begin
				i1_colour = 3'b000;
			end
		end
	end


    reg [7:0] u_x_counter;
	reg [6:0] u_y_counter;
	always @(posedge clock) 
	begin : u_xy
		if (!resetn || finish_change_letter) begin
			u_x_counter <= 8'd0;
            u_y_counter <= 7'd0;
			finish_u <= 1'b0;
		end
		else if (draw_u || draw_u1) begin
			if (u_x_counter == 8'd4) begin
				if (u_y_counter == 7'd6) begin
					finish_u <= 1'b1;
					u_y_counter <= 7'd0;
				end
				else begin
					finish_u <= 1'b0;
					u_y_counter <= u_y_counter + 1'b1;
				end
			u_x_counter <= 8'd0;
			end
			else begin
				finish_u <= 1'b0;
				u_x_counter <= u_x_counter + 1'b1;
			end
		end	
	end
	reg [2:0] u_colour, u1_colour;
	always @(*) 
    begin : u_colour_change
        if (!resetn) begin
            u_colour = 3'b000;
			u1_colour = 3'b000;
        end
		else if (draw_u) begin
			if (u_x_counter == 8'd0 || u_x_counter == 8'd3) begin
				if (u_y_counter == 8'd6) begin
					u_colour = 3'b000;
				end
				else begin
					u_colour = 3'b111;
				end
			end
			else if (u_y_counter == 8'd6) begin
				if (u_x_counter == 8'd1 || u_x_counter == 8'd2) begin
					u_colour = 3'b111;
				end
				else begin
					u_colour = 3'b000;
				end
			end
			else begin
				u_colour = 3'b000;
			end
		end
		else if (draw_u1) begin
			if (u_x_counter == 8'd0 || u_x_counter == 8'd3) begin
				if (u_y_counter == 8'd6) begin
					u1_colour = 3'b000;
				end
				else begin
					u1_colour = 3'b111;
				end
			end
			else if (u_y_counter == 8'd6) begin
				if (u_x_counter == 8'd1 || u_x_counter == 8'd2) begin
					u1_colour = 3'b111;
				end
				else begin
					u1_colour = 3'b000;
				end
			end
			else begin
				u1_colour = 3'b000;
			end
		end
	end


    reg [7:0] n_x_counter;
	reg [6:0] n_y_counter;
	always @(posedge clock) 
	begin : n_xy
		if (!resetn || finish_change_letter) begin
			n_x_counter <= 8'd0;
            n_y_counter <= 7'd0;
			finish_n <= 1'b0;
		end
		else if (draw_n || draw_n1 || draw_n2) begin
			if (n_x_counter == 8'd4) begin
				if (n_y_counter == 7'd6) begin
					finish_n <= 1'b1;
					n_y_counter <= 7'd0;
				end
				else begin
					finish_n <= 1'b0;
					n_y_counter <= n_y_counter + 1'b1;
				end
			n_x_counter <= 8'd0;
			end
			else begin
				finish_n <= 1'b0;
				n_x_counter <= n_x_counter + 1'b1;
			end
		end	
	end
    reg [2:0] n_colour;
	always @(*) 
    begin : n_colour_change
        if (!resetn) begin
            n_colour = 3'b000;
        end
		else if (draw_n) begin
			if (n_x_counter == 8'd0 || n_x_counter == 8'd4) begin
				n_colour = 3'b111;
			end
			else if (n_y_counter == 7'd0) begin
				if (n_x_counter == 8'd0 || n_x_counter == 8'd1 || n_x_counter == 8'd4) begin
					n_colour = 3'b111;
				end
				else begin
					n_colour = 3'b000;
				end
			end 
			else if (n_y_counter == 7'd6) begin
				if (n_x_counter == 8'd0 || n_x_counter == 8'd3 || n_x_counter == 8'd4) begin
					n_colour = 3'b111;
				end
				else begin
					n_colour = 3'b000;
				end
			end
			else if (n_x_counter == 8'd2) begin
				if (n_y_counter == 7'd0 || n_y_counter == 7'd6) begin
					n_colour = 3'b000;
				end
				else begin
					n_colour = 3'b111;
				end
			end
			else begin
				n_colour = 3'b000;
			end
		end
	end
	reg [2:0] n1_colour;
	always @(*) 
    begin : n1_colour_change
        if (!resetn) begin
            n1_colour = 3'b000;
        end
		else if (draw_n1) begin
			if (n_x_counter == 8'd0 || n_x_counter == 8'd4) begin
				n1_colour = 3'b111;
			end
			else if (n_y_counter == 7'd0) begin
				if (n_x_counter == 8'd0 || n_x_counter == 8'd1 || n_x_counter == 8'd4) begin
					n1_colour = 3'b111;
				end
				else begin
					n1_colour = 3'b000;
				end
			end 
			else if (n_y_counter == 7'd6) begin
				if (n_x_counter == 8'd0 || n_x_counter == 8'd3 || n_x_counter == 8'd4) begin
					n1_colour = 3'b111;
				end
				else begin
					n1_colour = 3'b000;
				end
			end
			else if (n_x_counter == 8'd2) begin
				if (n_y_counter == 7'd0 || n_y_counter == 7'd6) begin
					n1_colour = 3'b000;
				end
				else begin
					n1_colour = 3'b111;
				end
			end
			else begin
				n1_colour = 3'b000;
			end
		end
	end
	reg [2:0] n2_colour;
	always @(*) 
    begin : n2_colour_change
        if (!resetn) begin
            n2_colour = 3'b000;
        end
		else if (draw_n2) begin
			if (n_x_counter == 8'd0 || n_x_counter == 8'd4) begin
				n2_colour = 3'b111;
			end
			else if (n_y_counter == 7'd0) begin
				if (n_x_counter == 8'd0 || n_x_counter == 8'd1 || n_x_counter == 8'd4) begin
					n2_colour = 3'b111;
				end
				else begin
					n2_colour = 3'b000;
				end
			end 
			else if (n_y_counter == 7'd6) begin
				if (n_x_counter == 8'd0 || n_x_counter == 8'd3 || n_x_counter == 8'd4) begin
					n2_colour = 3'b111;
				end
				else begin
					n2_colour = 3'b000;
				end
			end
			else if (n_x_counter == 8'd2) begin
				if (n_y_counter == 7'd0 || n_y_counter == 7'd6) begin
					n2_colour = 3'b000;
				end
				else begin
					n2_colour = 3'b111;
				end
			end
			else begin
				n2_colour = 3'b000;
			end
		end
	end


    reg [7:0] t_x_counter;
	reg [6:0] t_y_counter;
	always @(posedge clock) 
	begin : t_xy
		if (!resetn || finish_change_letter) begin
			t_x_counter <= 8'd0;
            t_y_counter <= 7'd0;
			finish_t <= 1'b0;
		end
		else if (draw_t || draw_t1) begin
			if (t_x_counter == 8'd4) begin
				if (t_y_counter == 7'd7) begin
					finish_t <= 1'b1;
					t_y_counter <= 7'd0;
				end
				else begin
					finish_t <= 1'b0;
					t_y_counter <= t_y_counter + 1'b1;
				end
			t_x_counter <= 8'd0;
			end
			else begin
				finish_t <= 1'b0;
				t_x_counter <= t_x_counter + 1'b1;
			end
		end	
	end
	reg [2:0] t_colour, t1_colour;
	always @(*) 
    begin : t_colour_change
        if (!resetn) begin
            t_colour = 3'b000;
			t1_colour = 3'b000;
        end
		else if (draw_t) begin
			if (t_y_counter == 7'd1) begin
				if (t_x_counter >= 8'd0 && t_x_counter <= 8'd4) begin
					t_colour = 3'b111;
				end
				else begin
					t_colour = 3'b000;
				end
			end
			else if (t_x_counter == 8'd2) begin
				if (t_y_counter == 7'd0) begin
					t_colour = 3'b000;
				end
				else begin
					t_colour = 3'b111;
				end
			end
			else begin
				t_colour = 3'b000;
			end
		end
		else if (draw_t1) begin
			if (t_y_counter == 7'd1) begin
				if (t_x_counter >= 8'd0 && t_x_counter <= 8'd4) begin
					t1_colour = 3'b111;
				end
				else begin
					t1_colour = 3'b000;
				end
			end
			else if (t_x_counter == 8'd2) begin
				if (t_y_counter == 7'd0) begin
					t1_colour = 3'b000;
				end
				else begin
					t1_colour = 3'b111;
				end
			end
			else begin
				t1_colour = 3'b000;
			end
		end
	end


    reg [7:0] a_x_counter;
	reg [6:0] a_y_counter;
	always @(posedge clock) 
	begin : a_xy
		if (!resetn || finish_change_letter) begin
			a_x_counter <= 8'd0;
            a_y_counter <= 7'd0;
			finish_a <= 1'b0;
		end
		else if (draw_a || draw_a1) begin
			if (a_x_counter == 8'd4) begin
				if (a_y_counter == 7'd6) begin
					finish_a <= 1'b1;
					a_y_counter <= 7'd0;
				end
				else begin
					finish_a <= 1'b0;
					a_y_counter <= a_y_counter + 1'b1;
				end
			a_x_counter <= 8'd0;
			end
			else begin
				finish_a <= 1'b0;
				a_x_counter <= a_x_counter + 1'b1;
			end
		end	
	end
	reg [2:0] a_colour, a1_colour;
	always @(*) 
    begin : a_colour_change
        if (!resetn) begin
            a_colour = 3'b000;
			a1_colour = 3'b000;
        end
		else if (draw_a) begin
			if (a_x_counter == 8'd0 || a_x_counter == 8'd3) begin
				if (a_y_counter == 8'd0) begin
					a_colour = 3'b000;
				end
				else begin
					a_colour = 3'b111;
				end
			end
			else if (a_y_counter == 7'd0) begin
				if (a_x_counter == 8'd1 || a_x_counter == 8'd2) begin
					a_colour = 3'b111;
				end
				else begin
					a_colour = 3'b000;
				end
			end
			else if (a_y_counter == 7'd3) begin
				if (a_x_counter == 8'd4) begin
					a_colour = 3'b000;
				end
				else begin
					a_colour = 3'b111;
				end
			end
			else begin
				a_colour = 3'b000;
			end
		end
		else if (draw_a1) begin
			if (a_x_counter == 8'd0 || a_x_counter == 8'd3) begin
				if (a_y_counter == 8'd0) begin
					a1_colour = 3'b000;
				end
				else begin
					a1_colour = 3'b111;
				end
			end
			else if (a_y_counter == 7'd0) begin
				if (a_x_counter == 8'd1 || a_x_counter == 8'd2) begin
					a1_colour = 3'b111;
				end
				else begin
					a1_colour = 3'b000;
				end
			end
			else if (a_y_counter == 7'd3) begin
				if (a_x_counter == 8'd4) begin
					a1_colour = 3'b000;
				end
				else begin
					a1_colour = 3'b111;
				end
			end
			else begin
				a1_colour = 3'b000;
			end
		end
	end


    reg [7:0] r_x_counter;
	reg [6:0] r_y_counter;
	always @(posedge clock) 
	begin : r_xy
		if (!resetn || finish_change_letter) begin
			r_x_counter <= 8'd0;
            r_y_counter <= 7'd0;
			finish_r <= 1'b0;
		end
		else if (draw_r || draw_r1) begin
			if (r_x_counter == 8'd4) begin
				if (r_y_counter == 7'd6) begin
					finish_r <= 1'b1;
					r_y_counter <= 7'd0;
				end
				else begin
					finish_r <= 1'b0;
					r_y_counter <= r_y_counter + 1'b1;
				end
			r_x_counter <= 8'd0;
			end
			else begin
				finish_r <= 1'b0;
				r_x_counter <= r_x_counter + 1'b1;
			end
		end	
	end
	reg [2:0] r_colour, r1_colour;
	always @(*) 
    begin : r_colour_change
        if (!resetn) begin
            r_colour = 3'b000;
			r1_colour = 3'b000;
        end
		else if (draw_r) begin
			if (r_x_counter == 8'd0) begin
				r_colour = 3'b111;
			end
			else if (r_x_counter == 8'd3) begin
				if (r_y_counter == 7'd1 || r_y_counter == 7'd2 || r_y_counter == 7'd6) begin
					r_colour = 3'b111;
				end
				else begin
					r_colour = 3'b000;
				end
			end
			else if (r_y_counter == 7'd0 || r_y_counter == 7'd3) begin
				if (r_x_counter >= 8'd0 && r_x_counter <= 8'd2) begin
					r_colour = 3'b111;
				end
				else begin
					r_colour = 3'b000;
				end
			end
			else if (r_y_counter == 7'd4) begin
				if (r_x_counter == 7'd0 || r_x_counter == 7'd1) begin
					r_colour = 3'b111;
				end
				else begin
					r_colour = 3'b000;
				end
			end
			else if (r_y_counter == 7'd5) begin
				if (r_x_counter == 7'd0 || r_x_counter == 7'd2) begin
					r_colour = 3'b111;
				end
				else begin
					r_colour = 3'b000;
				end
			end
			else begin
				r_colour = 3'b000;
			end
		end
		else if (draw_r1) begin
			if (r_x_counter == 8'd0) begin
				r1_colour = 3'b111;
			end
			else if (r_x_counter == 8'd3) begin
				if (r_y_counter == 7'd1 || r_y_counter == 7'd2 || r_y_counter == 7'd6) begin
					r1_colour = 3'b111;
				end
				else begin
					r1_colour = 3'b000;
				end
			end
			else if (r_y_counter == 7'd0 || r_y_counter == 7'd3) begin
				if (r_x_counter >= 8'd0 && r_x_counter <= 8'd2) begin
					r1_colour = 3'b111;
				end
				else begin
					r1_colour = 3'b000;
				end
			end
			else if (r_y_counter == 7'd4) begin
				if (r_x_counter == 7'd0 || r_x_counter == 7'd1) begin
					r1_colour = 3'b111;
				end
				else begin
					r1_colour = 3'b000;
				end
			end
			else if (r_y_counter == 7'd5) begin
				if (r_x_counter == 7'd0 || r_x_counter == 7'd2) begin
					r1_colour = 3'b111;
				end
				else begin
					r1_colour = 3'b000;
				end
			end
			else begin
				r1_colour = 3'b000;
			end
		end
	end


    //count letter pixel
	reg [5:0] count_letter_pixel;
	always @(posedge clock)
	begin : letter_pixel_counter
		if (!resetn || finish_change_letter) begin
			count_letter_pixel <= 6'd0;
			finish_p <= 1'b0;
			finish_k <= 1'b0;
			finish_c <= 1'b0;
			finish_h <= 1'b0;
			finish_g <= 1'b0;
			finish_s <= 1'b0;
		end
		else if (draw_p || draw_k || draw_c || 
				draw_h || draw_g || draw_s) begin
			if (count_letter_pixel == 6'd34) begin
				finish_p <= 1'b1;
				finish_k <= 1'b1;
				finish_c <= 1'b1;
				finish_h <= 1'b1;
				finish_g <= 1'b1;
				finish_s <= 1'b1;
				count_letter_pixel <= 6'd0;
			end
			else begin
				finish_p <= 1'b0;
				finish_k <= 1'b0;
				finish_c <= 1'b0;
				finish_h <= 1'b0;
				finish_g <= 1'b0;
				finish_s <= 1'b0;
				count_letter_pixel <= count_letter_pixel + 1'b1;
			end
		end
	end


	// Position output
	reg [7:0] ground_x_pos_counter;
    reg [6:0] ground_y_pos_counter;

	// The xy register
	always @(posedge clock)
	begin : xy_register
		if (!resetn) begin
			x_out <= 8'd0;
			y_out <= 7'd0;
		end
		else if (draw_ground) begin
			x_out <= ground_x_pos_counter;
			y_out <= ground_y_pos_counter;
		end
        else if (draw_start_scene) begin
			x_out <= 8'd42 + game_start_x_counter;
			y_out <= 7'd24 + game_start_y_counter;
		end
		else if (draw_p) begin
			x_out <= 8'd44 + game_start_x_counter;
			y_out <= 7'd29 + game_start_y_counter;
		end
		else if (draw_i) begin
			x_out <= 8'd49 + i_x_counter;
			y_out <= 7'd29 + i_y_counter;
		end
		else if (draw_i1) begin
			x_out <= 8'd105 + i_x_counter;
			y_out <= 7'd29 + i_y_counter;
		end
		else if (draw_k) begin
			x_out <= 8'd53 + game_start_x_counter;
			y_out <= 7'd29 + game_start_y_counter;
		end
		else if (draw_a) begin
			x_out <= 8'd58 + a_x_counter;
			y_out <= 7'd29 + a_y_counter;
		end
		else if (draw_c) begin
			x_out <= 8'd63 + game_start_x_counter;
			y_out <= 7'd29 + game_start_y_counter;
		end
		else if (draw_h) begin
			x_out <= 8'd68 + game_start_x_counter;
			y_out <= 7'd29 + game_start_y_counter;
		end
		else if (draw_u) begin
			x_out <= 8'd73 + u_x_counter;
			y_out <= 7'd29 + u_y_counter;
		end
		else if (draw_u1) begin
			x_out <= 8'd88 + u_x_counter;
			y_out <= 7'd29 + u_y_counter;
		end
		else if (draw_r) begin
			x_out <= 8'd83 + r_x_counter;
			y_out <= 7'd29 + r_y_counter;
		end
		else if (draw_n) begin
			x_out <= 8'd93 + n_x_counter;
			y_out <= 7'd29 + n_y_counter;
		end
		else if (draw_n1) begin
			x_out <= 8'd99 + n_x_counter;
			y_out <= 7'd29 + n_y_counter;
		end
		else if (draw_n2) begin
			x_out <= 8'd109 + n_x_counter;
			y_out <= 7'd29 + n_y_counter;
		end
		else if (draw_g) begin
			x_out <= 8'd115 + game_start_x_counter;
			y_out <= 7'd29 + game_start_y_counter;
		end
		else if (draw_s) begin
			x_out <= 8'd68 + game_start_x_counter;
			y_out <= 7'd70 + game_start_y_counter;
		end
		else if (draw_t) begin
			x_out <= 8'd73 + t_x_counter;
			y_out <= 7'd69 + t_y_counter;
		end
		else if (draw_a1) begin
			x_out <= 8'd79 + a_x_counter;
			y_out <= 7'd70 + a_y_counter;
		end
		else if (draw_r1) begin
			x_out <= 8'd84 + r_x_counter;
			y_out <= 7'd70 + r_y_counter;
		end
		else if (draw_t1) begin
			x_out <= 8'd89 + t_x_counter;
			y_out <= 7'd69 + t_y_counter;
		end
		else if (draw_ground_low) begin
			x_out <= ground_x_pos_counter;
			y_out <= ground_y_pos_counter;
		end
		else if (draw_ball) begin
			x_out <= 8'd10 + pikachu_x_dead_counter;
			y_out <= 7'd109 + pikachu_y_dead_counter;
		end
		else if (en_erase_start) begin
			x_out <= 8'd67 + e_start_counter_x;
			y_out <= 7'd70 + e_start_counter_y;
		end
		else if (draw_death) begin
			if (change_status) begin
				x_out <= pikachu_move_x + 1'b1 + pikachu_x_dead_counter;
			end
			else if (!change_status) begin
				x_out <= pikachu_move_x + pikachu_x_dead_counter;
			end
			y_out <= pikachu_move_y + 7'd1 + pikachu_y_dead_counter;
		end
		else if (draw_pikachu) begin
			if (draw_pikachu_up) begin
				x_out <= pikachu_move_x + pikachu_x_up_counter;
				y_out <= pikachu_move_y + pikachu_y_up_counter;
			end
			else if (draw_pikachu_down) begin
				x_out <= pikachu_move_x + pikachu_x_down_counter;
				y_out <= 7'd111 + pikachu_y_down_counter;
			end
		end
		else if (draw_holding_bar) begin
			x_out <= 8'd5;
			y_out <= 7'd103 - holding;
		end
		else if (draw_new_death) begin
			x_out <= dead_move_x + pikachu_x_dead_counter;
			y_out <= dead_move_y + pikachu_y_dead_counter;
		end
		else if (draw_cloud) begin
			x_out <= cloud_move_x - cloud_x;
			y_out <= 7'd60 + cloud_y;
		end
		else if (draw_bird) begin
			x_out <= bird_move_x + bird_x;
			y_out <= bird_move_y + bird_y;
		end
		else if (draw_tree) begin
			x_out <= tree_move_x + tree_x;
			y_out <= tree_move_y + tree_y;
		end
		else if (en_erase) begin
			x_out <= erase_counter_x;
			y_out <= erase_counter_y;
		end
	end

	
	// The background colour register
	always @(posedge clock)
	begin : background_colour_register
		if (!resetn) begin
			colour_out <= 3'b000;
		end
        else if (draw_start_scene || draw_p || draw_k || 
				draw_c || draw_h || draw_g || draw_s) begin
			colour_out <= game_start_colour;
		end
		else if (draw_t) begin
			colour_out <= t_colour;
		end
		else if (draw_t1) begin
			colour_out <= t1_colour;
		end
		else if (draw_a) begin
			colour_out <= a_colour;
		end
		else if (draw_a1) begin
			colour_out <= a1_colour;
		end
		else if (draw_r) begin
			colour_out <= r_colour;
		end
		else if (draw_r1) begin
			colour_out <= r1_colour;
		end
		else if (draw_i) begin
			colour_out <= i_colour;
		end
		else if (draw_i1) begin
			colour_out <= i1_colour;
		end
		else if (draw_u) begin
			colour_out <= u_colour;
		end
		else if (draw_u1) begin
			colour_out <= u1_colour;
		end
        else if (draw_n) begin
			colour_out <= n_colour;
		end
		else if (draw_n1) begin
			colour_out <= n1_colour;
		end
		else if (draw_n2) begin
			colour_out <= n2_colour;
		end
		else if (draw_ground_low) begin
			colour_out <= low_colour;
		end
		else if (draw_ball) begin
			colour_out <= ball_colour;
		end
		else if (en_erase_start) begin
			colour_out <= 3'b000;
		end
		else if (draw_ground) begin
			colour_out <= background_colour;
		end
		else if (draw_death || draw_new_death) begin
			colour_out <= pikachu_dead_colour;
		end
		else if (draw_pikachu) begin
			if (draw_pikachu_up) begin
				colour_out <= pikachu_up_colour;
			end
			else if (draw_pikachu_down) begin
				colour_out <= pikachu_down_colour;
			end
		end
		else if (draw_holding_bar) begin
			colour_out <= holding_colour;
		end
		else if (draw_cloud) begin
			colour_out <= cloud_colour;
		end
		else if (draw_tree) begin
			colour_out <= 3'b010;
		end
		else if (draw_bird) begin
			colour_out <= bird_colour;
		end
		else if (en_erase) begin
			if (erase_counter_y < 7'd104 && erase_counter_y > 7'd93 && erase_counter_x == 8'd5) begin
				colour_out <= 3'b000;
			end
			else 
				colour_out <= erase_colour;
		end
	end

	// erase counter xy
	reg [7:0] erase_counter_x;
	reg [6:0] erase_counter_y;
	always @(posedge clock)
	begin : erase_counter
		if (!resetn) begin
			erase_counter_x <= 8'd0;
			erase_counter_y <= 7'd0;
		end
		else if (erase_counter_y == 7'd120) begin
			erase_counter_y <= 7'd0;
			finish_erase <= 1'b1;
		end
		else if (en_erase) begin
			if(erase_counter_x == 8'd159) begin
				erase_counter_x <= 8'd0;
				erase_counter_y <= erase_counter_y + 1'b1;
			end
			else begin
				erase_counter_x <= erase_counter_x + 1'b1;
				finish_erase <= 1'b0;
			end
		end
	end
	reg [2:0] erase_colour;
	// The colour change for the erase
	always @(*)
	begin : erase_colour_change
        if (!resetn) begin
            erase_colour = 3'b011;
        end
		else if (erase_counter_y > 7'd115)
			erase_colour = 3'b111;
		else 
			erase_colour = 3'b011;
	end


    // erase counter xy for start
	reg [7:0] e_start_counter_x;
	reg [6:0] e_start_counter_y;
	always @(posedge clock)
	begin : e_start_counter
		if (!resetn) begin
			e_start_counter_x <= 8'd0;
			e_start_counter_y <= 7'd0;
		end
		else if (e_start_counter_y == 7'd7) begin
			e_start_counter_y <= 7'd0;
			finish_erase_start <= 1'b1;
		end
		else if (en_erase_start) begin
			if(e_start_counter_x == 8'd27) begin
				e_start_counter_x <= 8'd0;
				e_start_counter_y <= e_start_counter_y + 1'b1;
			end
			else begin
				e_start_counter_x <= e_start_counter_x + 1'b1;
				finish_erase_start <= 1'b0;
			end
		end
	end


	// The ground xy counter
	always @(posedge clock)
	begin : ground_xy_counter
		if (!resetn) begin
			ground_x_pos_counter <= 8'd0;
            ground_y_pos_counter <= 7'd0;
			finish_ground <= 1'b0;
            finish_ground_low <= 1'b0;
		end
        else if (draw_ground_low) begin
			if (ground_x_pos_counter == 8'd160) begin 
                if (ground_y_pos_counter == 7'd120) begin
                    finish_ground_low <= 1'b1;
                    ground_y_pos_counter <= 7'd0;
                end
                else begin
                    finish_ground_low <= 1'b0;
                    ground_y_pos_counter <= ground_y_pos_counter + 1'b1;
                end
				ground_x_pos_counter <= 8'd0;
			end
			else begin
				finish_ground_low <= 1'b0;
				ground_x_pos_counter <= ground_x_pos_counter + 1'b1;
			end
		end
		else if (draw_ground) begin
			if (ground_x_pos_counter == 8'd160) begin 
                if (ground_y_pos_counter == 7'd120) begin
                    finish_ground <= 1'b1;
                    ground_y_pos_counter <= 7'd0;
                end
                else begin
                    finish_ground <= 1'b0;
                    ground_y_pos_counter <= ground_y_pos_counter + 1'b1;
                end
				ground_x_pos_counter <= 8'd0;
			end
			else begin
				finish_ground <= 1'b0;
				ground_x_pos_counter <= ground_x_pos_counter + 1'b1;
			end
		end
	end
    reg [2:0] low_colour;
	// The colour change for the ground_low
	always @(*)
	begin : low_colour_change
        if (!resetn) begin
            low_colour = 3'b000;
        end
		else if (ground_y_pos_counter > 7'd115)
			low_colour = 3'b111;
		else 
			low_colour = 3'b000;
	end
	reg [2:0] background_colour;
	// The colour change for the background
	always @(*)
	begin : background_colour_change
        if (!resetn) begin
            background_colour = 3'b011;
        end
		else if (ground_y_pos_counter > 7'd115)
			background_colour = 3'b111;
		else if (ground_x_pos_counter == 8'd5) begin
			if (ground_y_pos_counter < 7'd104 && ground_y_pos_counter > 7'd93) begin
				background_colour <= 3'b000;
			end
			else 
				background_colour <= 3'b011;
		end
		else 
			background_colour = 3'b011;
	end


	reg [2:0] holding_colour;
	// The colour change for the holding bar
	always @(*) 
    begin : holding_bar_colour_change
        if (!resetn) begin
            holding_colour <= 3'b000;
        end
        else if (holding >= 5'd0 && holding <= 5'd3) begin
			holding_colour <= 3'b010;
        end
		else if (holding >= 5'd4 && holding <= 5'd6) begin
			holding_colour <= 3'b110;
		end
		else if (holding >= 5'd7 && holding <= 5'd8) begin 
			holding_colour <= 3'b100;
		end
    end


	// count totoal pixel for the pikachu
	reg [5:0] count_pixel;
	always @(posedge clock)
	begin : pixel_counter
		if (!resetn || stop) begin
			count_pixel <= 6'd0;
			finish_pikachu <= 1'b0;
			finish_cloud <= 1'b0;
			finish_death <= 1'b0;
			finish_bird <= 1'b0;
		end
		//if (stop) begin
		//	count_pixel <= 6'd0;
		//end
		else if (draw_pikachu || draw_cloud) begin
			if (count_pixel == 6'd39) begin
				finish_cloud <= 1'b1;
				finish_pikachu <= 1'b1;
				count_pixel <= 6'd0;
			end
			else begin
				finish_cloud <= 1'b0;
				finish_pikachu <= 1'b0;
				count_pixel <= count_pixel + 1'b1;
			end
		end
		else if (draw_ball || draw_death || draw_new_death) begin
			if (count_pixel == 6'd34) begin
				finish_death <= 1'b1;
				count_pixel <= 6'd0;
			end
			else begin
				finish_death <= 1'b0;
				count_pixel <= count_pixel + 1'b1;
			end
		end
		else if (draw_bird) begin
			if (count_pixel == 6'd41) begin
				finish_bird <= 1'b1;
				count_pixel <= 6'd0;
			end
			else begin
				finish_bird <= 1'b0;
				count_pixel <= count_pixel + 1'b1;
			end
		end
	end


	// dead pikachu xy
	reg [7:0] pikachu_x_dead_counter;
    reg [6:0] pikachu_y_dead_counter;
	always @(posedge clock)
	begin
		if (!resetn) begin
			pikachu_x_dead_counter <= 8'd0;
            pikachu_y_dead_counter <= 7'd0;
		end
		else if(draw_ball || draw_death || draw_new_death) begin
			if (count_pixel >= 6'd0 && count_pixel <= 6'd4) begin
				pikachu_x_dead_counter <= count_pixel;
				pikachu_y_dead_counter <= 7'd0;
			end
			else if (count_pixel >= 6'd5 && count_pixel <= 6'd9) begin
				pikachu_x_dead_counter <= count_pixel - 6'd5;
				pikachu_y_dead_counter <= 7'd1;
			end
			else if (count_pixel >= 6'd10 && count_pixel <= 6'd14) begin
				pikachu_x_dead_counter <= count_pixel - 6'd10;
				pikachu_y_dead_counter <= 7'd2;
			end
			else if (count_pixel >= 6'd15 && count_pixel <= 6'd19) begin
				pikachu_x_dead_counter <= count_pixel - 6'd15;
				pikachu_y_dead_counter <= 7'd3;
			end
			else if (count_pixel >= 6'd20 && count_pixel <= 6'd24) begin
				pikachu_x_dead_counter <= count_pixel - 6'd20;
				pikachu_y_dead_counter <= 7'd4;
			end
			else if (count_pixel >= 6'd25 && count_pixel <= 6'd29) begin
				pikachu_x_dead_counter <= count_pixel - 6'd25;
				pikachu_y_dead_counter <= 7'd5;
			end
			else if (count_pixel >= 6'd30 && count_pixel <= 6'd34) begin
				pikachu_x_dead_counter <= count_pixel - 6'd30;
				pikachu_y_dead_counter <= 7'd6;
			end
		end
	end
	reg [2:0] pikachu_dead_colour;
	// The colour change for the dead pikachu
    always @(*) 
    begin : dead_pikachu_colour_change
        if (!resetn) begin
            pikachu_dead_colour = 3'b111;
        end
        else if (pikachu_y_dead_counter == 7'd0) begin
			if (pikachu_x_dead_counter == 8'd0 || pikachu_x_dead_counter == 8'd4) begin
				pikachu_dead_colour = 3'b011;
			end
            else begin 
                pikachu_dead_colour = 3'b100;
            end
        end
		else if (pikachu_y_dead_counter == 7'd1) begin
			pikachu_dead_colour = 3'b100;
        end
        else if (pikachu_y_dead_counter == 7'd2) begin
            if (pikachu_x_dead_counter == 8'd2) begin
				pikachu_dead_colour = 3'b000;
			end
            else begin
                pikachu_dead_colour = 3'b100;
            end
        end
        else if (pikachu_y_dead_counter == 7'd3) begin
            if (pikachu_x_dead_counter == 8'd2) begin
				pikachu_dead_colour = 3'b111;
			end
            else begin
                pikachu_dead_colour = 3'b000;
            end
        end
        else if (pikachu_y_dead_counter == 7'd4) begin
			if (pikachu_x_dead_counter == 8'd2) begin
				pikachu_dead_colour = 3'b000;
			end
			else begin
				pikachu_dead_colour = 3'b111;
			end
        end
		else if (pikachu_y_dead_counter == 7'd5) begin
			pikachu_dead_colour = 3'b111;
        end
        else if (pikachu_y_dead_counter == 7'd6) begin
            if (pikachu_x_dead_counter == 8'd0 || pikachu_x_dead_counter == 8'd4) begin
                pikachu_dead_colour = 3'b011;
            end
            else begin
                pikachu_dead_colour = 3'b111;
            end
        end
    end
    reg [2:0] ball_colour;
	// The colour change for the ball
    always @(*) 
    begin : ball_colour_change
        if (!resetn) begin
            ball_colour = 3'b111;
        end
        else if (pikachu_y_dead_counter == 7'd0) begin
			if (pikachu_x_dead_counter == 8'd0 || pikachu_x_dead_counter == 8'd4) begin
				ball_colour = 3'b000;
			end
            else begin 
                ball_colour = 3'b100;
            end
        end
		else if (pikachu_y_dead_counter == 7'd1) begin
			ball_colour = 3'b100;
        end
        else if (pikachu_y_dead_counter == 7'd2) begin
            if (pikachu_x_dead_counter == 8'd2) begin
				ball_colour = 3'b000;
			end
            else begin
                ball_colour = 3'b100;
            end
        end
        else if (pikachu_y_dead_counter == 7'd3) begin
            if (pikachu_x_dead_counter == 8'd2) begin
				ball_colour = 3'b111;
			end
            else begin
                ball_colour = 3'b000;
            end
        end
        else if (pikachu_y_dead_counter == 7'd4) begin
			if (pikachu_x_dead_counter == 8'd2) begin
				ball_colour = 3'b000;
			end
			else begin
				ball_colour = 3'b111;
			end
        end
		else if (pikachu_y_dead_counter == 7'd5) begin
			ball_colour = 3'b111;
        end
        else if (pikachu_y_dead_counter == 7'd6) begin
            if (pikachu_x_dead_counter == 8'd0 || pikachu_x_dead_counter == 8'd4) begin
                ball_colour = 3'b000;
            end
            else begin
                ball_colour = 3'b111;
            end
        end
    end

	// standing pikachu xy
    reg [7:0] pikachu_x_up_counter;
    reg [6:0] pikachu_y_up_counter;
	always @(posedge clock)
	begin
		if (!resetn) begin
			pikachu_x_up_counter <= 8'd0;
            pikachu_y_up_counter <= 7'd0;
		end
		else if(draw_pikachu_up) begin
			if (count_pixel >= 6'd0 && count_pixel <= 6'd4) begin
				pikachu_x_up_counter <= count_pixel;
				pikachu_y_up_counter <= 7'd0;
			end
			else if (count_pixel >= 6'd5 && count_pixel <= 6'd9) begin
				pikachu_x_up_counter <= count_pixel - 3'd5;
				pikachu_y_up_counter <= 7'd1;
			end
			else if (count_pixel >= 6'd10 && count_pixel <= 6'd14) begin
				pikachu_x_up_counter <= count_pixel - 4'd10;
				pikachu_y_up_counter <= 7'd2;
			end
			else if (count_pixel >= 6'd15 && count_pixel <= 6'd19) begin
				pikachu_x_up_counter <= count_pixel - 4'd15;
				pikachu_y_up_counter <= 7'd3;
			end
			else if (count_pixel >= 6'd20 && count_pixel <= 6'd24) begin
				pikachu_x_up_counter <= count_pixel - 5'd20;
				pikachu_y_up_counter <= 7'd4;
			end
			else if (count_pixel >= 6'd25 && count_pixel <= 6'd29) begin
				pikachu_x_up_counter <= count_pixel - 5'd25;
				pikachu_y_up_counter <= 7'd5;
			end
			else if (count_pixel >= 6'd30 && count_pixel <= 6'd34) begin
				pikachu_x_up_counter <= count_pixel - 5'd30;
				pikachu_y_up_counter <= 7'd6;
			end
			else if (count_pixel >= 6'd35 && count_pixel <= 6'd39) begin
				pikachu_x_up_counter <= count_pixel - 6'd35;
				pikachu_y_up_counter <= 7'd7;
			end
		end
	end
	

    reg [2:0] pikachu_up_colour;
	// The colour change for the standing pikachu
    always @(*) 
    begin : standing_pikachu_colour_change
        if (!resetn) begin
            pikachu_up_colour = 3'b011;
        end
        else if (pikachu_y_up_counter == 7'd0) begin
            if (pikachu_x_up_counter == 8'd2 || pikachu_x_up_counter == 8'd4) begin
                pikachu_up_colour = 3'b000;
            end
            else begin 
                pikachu_up_colour = 3'b011;
            end
        end
        else if (pikachu_y_up_counter == 7'd1) begin
            if (pikachu_x_up_counter == 8'd0 || pikachu_x_up_counter == 8'd1) begin
				pikachu_up_colour = 3'b011;
			end
            else begin
                pikachu_up_colour = 3'b110;
            end
        end
        else if (pikachu_y_up_counter == 7'd2) begin
            if (pikachu_x_up_counter == 8'd0 || pikachu_x_up_counter == 8'd1) begin
				pikachu_up_colour = 3'b011;
			end
            else if (pikachu_x_up_counter == 8'd2 || pikachu_x_up_counter == 8'd4) begin
                pikachu_up_colour = 3'b000;
            end
            else begin
                pikachu_up_colour = 3'b110;
            end
        end
        else if (pikachu_y_up_counter == 7'd3) begin
            if (pikachu_x_up_counter == 8'd0 || pikachu_x_up_counter == 8'd1) begin
				pikachu_up_colour = 3'b011;
			end
            else if (pikachu_x_up_counter == 8'd2 || pikachu_x_up_counter == 8'd4) begin
                pikachu_up_colour = 3'b100;
            end
            else begin
                pikachu_up_colour = 3'b111;
            end
        end
        else if (pikachu_y_up_counter == 7'd4) begin
            if (pikachu_x_up_counter == 8'd1) begin
				pikachu_up_colour = 3'b011;
			end
            else begin
                pikachu_up_colour = 3'b110;
            end
        end
		else if (pikachu_y_up_counter == 7'd5) begin
        	pikachu_up_colour = 3'b110;
		end
        else if (pikachu_y_up_counter == 7'd6) begin
            if (pikachu_x_up_counter == 8'd0) begin
				pikachu_up_colour = 3'b011;
			end
            else if (pikachu_x_up_counter == 8'd1) begin
				pikachu_up_colour = 3'b000;
			end
            else begin
                pikachu_up_colour = 3'b110;
            end
        end
        else if (pikachu_y_up_counter == 7'd7) begin
            if (pikachu_x_up_counter == 8'd0 || pikachu_x_up_counter == 8'd1) begin
				pikachu_up_colour = 3'b011;
			end
            else begin
                pikachu_up_colour = 3'b110;
            end 
        end

    end


	// squating pikachu xy
    reg [7:0] pikachu_x_down_counter;
    reg [6:0] pikachu_y_down_counter;
	always @(posedge clock)
	begin
		if (!resetn) begin
			pikachu_x_down_counter <= 8'd0;
            pikachu_y_down_counter <= 7'd0;
		end
		else if(draw_pikachu_down) begin
			if (count_pixel >= 6'd0 && count_pixel <= 6'd7) begin
				pikachu_x_down_counter <= count_pixel;
				pikachu_y_down_counter <= 7'd0;
			end
			else if (count_pixel >= 6'd8 && count_pixel <= 6'd15) begin
				pikachu_x_down_counter <= count_pixel - 6'd8;
				pikachu_y_down_counter <= 7'd1;
			end
			else if (count_pixel >= 6'd16 && count_pixel <= 6'd23) begin
				pikachu_x_down_counter <= count_pixel - 6'd16;
				pikachu_y_down_counter <= 7'd2;
			end
			else if (count_pixel >= 6'd24 && count_pixel <= 6'd31) begin
				pikachu_x_down_counter <= count_pixel - 6'd24;
				pikachu_y_down_counter <= 7'd3;
			end
			else if (count_pixel >= 6'd32 && count_pixel <= 6'd39) begin
				pikachu_x_down_counter <= count_pixel - 6'd32;
				pikachu_y_down_counter <= 7'd4;
			end
		end
	end
	reg [2:0] pikachu_down_colour;
	// The colour change for the squating pikachu
    always @(*) 
    begin : squating_pikachu_colour_change
        if (!resetn) begin
            pikachu_down_colour = 3'b011;
        end
        else if (pikachu_y_down_counter == 7'd0) begin
			if (pikachu_x_down_counter == 8'd0) begin
				pikachu_down_colour = 3'b011;
			end
            else if (pikachu_x_down_counter == 8'd1) begin 
                pikachu_down_colour = 3'b110;
            end
            else if (pikachu_x_down_counter == 8'd5 || pikachu_x_down_counter == 8'd7) begin
                pikachu_down_colour = 3'b000;
            end
            else begin 
                pikachu_down_colour = 3'b011;
            end
        end
        else if (pikachu_y_down_counter == 7'd1) begin
            if (pikachu_x_down_counter == 8'd0) begin
				pikachu_down_colour = 3'b011;
			end
            else if (pikachu_x_down_counter == 8'd3) begin
                pikachu_down_colour = 3'b000;
            end
            else begin
                pikachu_down_colour = 3'b110;
            end
        end
        else if (pikachu_y_down_counter == 7'd2) begin
            if (pikachu_x_down_counter == 8'd0) begin
				pikachu_down_colour = 3'b011;
			end
            else if (pikachu_x_down_counter == 8'd5 || pikachu_x_down_counter == 8'd7) begin
                pikachu_down_colour = 3'b000;
            end
            else begin
                pikachu_down_colour = 3'b110;
            end
        end
        else if (pikachu_y_down_counter == 7'd3) begin
			if (pikachu_x_down_counter == 8'd0) begin
				pikachu_down_colour = 3'b011;
			end
			else if (pikachu_x_down_counter == 8'd1) begin
				pikachu_down_colour = 3'b000;
			end
			else if (pikachu_x_down_counter == 8'd5 || pikachu_x_down_counter == 8'd7) begin
				pikachu_down_colour = 3'b100;
			end
			else if (pikachu_x_down_counter == 8'd6) begin
				pikachu_down_colour = 3'b111;
			end
			else begin
				pikachu_down_colour = 3'b110;
			end
        end
        else if (pikachu_y_down_counter == 7'd4) begin
            if (pikachu_x_down_counter == 8'd0) begin
				pikachu_down_colour = 3'b011;
			end
            else if (pikachu_x_down_counter == 8'd1) begin
                pikachu_down_colour = 3'b011;
            end
            else begin
                pikachu_down_colour = 3'b110;
            end
        end
    end


	// cloud counter
	reg [7:0] cloud_x;
	reg [6:0] cloud_y;
	always @(posedge clock) begin
		if (!resetn) begin
			cloud_x <= 8'd0;
            cloud_y <= 7'd0;
		end
		else if(draw_cloud) begin
			if (count_pixel >= 6'd0 && count_pixel <= 6'd9) begin
				cloud_x <= count_pixel;
				cloud_y <= 7'd0;
			end
			else if (count_pixel >= 6'd10 && count_pixel <= 6'd19) begin
				cloud_x <= count_pixel - 6'd10;
				cloud_y <= 7'd1;
			end
			else if (count_pixel >= 6'd20 && count_pixel <= 6'd29) begin
				cloud_x <= count_pixel - 6'd20;
				cloud_y <= 7'd2;
			end
			else if (count_pixel >= 6'd30 && count_pixel <= 6'd39) begin
				cloud_x <= count_pixel - 6'd30;
				cloud_y <= 7'd3;
			end
		end
	end
	reg [2:0] cloud_colour;
	// The colour change for the cloud
    always @(*) 
    begin : cloud_colour_change
        if (!resetn) begin
            cloud_colour = 3'b011;
        end
        else if (cloud_y == 7'd0) begin
			if (cloud_x >= 8'd3 && cloud_x <= 8'd6) begin
				cloud_colour = 3'b111;
			end
            else begin 
                cloud_colour = 3'b011;
            end
        end
        else if (cloud_y == 7'd1) begin
            if (cloud_x == 8'd0 || cloud_x == 8'd9) begin
				cloud_colour = 3'b011;
			end
            else begin
                cloud_colour = 3'b111;
            end
        end
        else if (cloud_y == 7'd2) begin
            cloud_colour = 3'b111;
        end
        else if (cloud_y == 7'd3) begin
			cloud_colour = 3'b111;
        end
    end


	// tree counter
	reg [7:0] tree_x;
	reg [6:0] tree_y;
	always @(posedge clock) begin
		if (!resetn) begin
			tree_x <= 8'd0;
            tree_y <= 7'd0;
			finish_tree <= 1'b0;
		end
		if (stop) begin
			tree_x <= 8'd0;
			tree_y <= 7'd0;
		end
		else if(draw_tree) begin
			if (tree_x == 8'd3) begin
				if (tree_move_y + tree_y == 7'd115) begin
					finish_tree <= 1'b1;
					tree_y <= 7'd0;
				end
				else begin
					finish_tree <= 1'b0;
					tree_y <= tree_y + 1'b1;
				end
				tree_x <= 8'd0;
			end
			else begin
				finish_tree <= 1'b0;
				tree_x <= tree_x + 1'b1;
			end
		end
	end


	// bird counter
	reg [7:0] bird_x;
	reg [6:0] bird_y;
	always @(posedge clock) begin
		if (!resetn) begin
			bird_x <= 8'd0;
            bird_y <= 7'd0;
		end
		else if(draw_bird) begin
			if (count_pixel >= 6'd0 && count_pixel <= 6'd6) begin
				bird_x <= count_pixel;
				bird_y <= 7'd0;
			end
			else if (count_pixel >= 6'd7 && count_pixel <= 6'd13) begin
				bird_x <= count_pixel - 6'd7;
				bird_y <= 7'd1;
			end
			else if (count_pixel >= 6'd14 && count_pixel <= 6'd20) begin
				bird_x <= count_pixel - 6'd14;
				bird_y <= 7'd2;
			end
			else if (count_pixel >= 6'd21 && count_pixel <= 6'd27) begin
				bird_x <= count_pixel - 6'd21;
				bird_y <= 7'd3;
			end
			else if (count_pixel >= 6'd28 && count_pixel <= 6'd34) begin
				bird_x <= count_pixel - 6'd28;
				bird_y <= 7'd4;
			end
			else if (count_pixel >= 6'd35 && count_pixel <= 6'd41) begin
				bird_x <= count_pixel - 6'd35;
				bird_y <= 7'd5;
			end
		end
	end
	reg [2:0] bird_colour;
	// The colour change for the bird
    always @(*) 
    begin : bird_colour_change
        if (!resetn) begin
            bird_colour = 3'b011;
        end
        else if (bird_y == 7'd0) begin
			if (bird_x >= 8'd2 && bird_x <= 8'd4) begin
				bird_colour = 3'b000;
			end
            else begin 
                bird_colour = 3'b011;
            end
        end
        else if (bird_y == 7'd1) begin
            if (bird_x == 8'd3) begin
				bird_colour = 3'b000;
			end
			else begin
				bird_colour = 3'b011;
			end
        end
        else if (bird_y == 7'd2) begin
			if (bird_x == 8'd0) begin
				bird_colour = 3'b100;
			end
			else if (bird_x == 8'd1 || bird_x == 8'd5) begin
				bird_colour = 3'b011;
			end
			else if (bird_x == 8'd6) begin
				bird_colour = 3'b001;
			end
            else begin 
                bird_colour = 3'b111;
            end
        end
        else if (bird_y == 7'd3) begin
			if (bird_x == 8'd0 || bird_x == 8'd6) begin
				bird_colour = 3'b011;
			end
			else if (bird_x == 8'd1 || bird_x == 8'd5) begin
				bird_colour = 3'b000;
			end
			else if (bird_x == 8'd3) begin
				bird_colour = 3'b000;
			end
			else begin
				bird_colour = 3'b111;
			end
        end
		else if (bird_y == 7'd4) begin
			if (bird_x == 8'd0 || bird_x == 8'd6) begin
				bird_colour = 3'b011;
			end
			else if (bird_x == 8'd1 || bird_x == 8'd5) begin
				bird_colour = 3'b000;
			end
			else if (bird_x == 8'd3) begin
				bird_colour = 3'b111;
			end
			else begin
				bird_colour = 3'b111;
			end
		end
		else if (bird_y == 7'd5) begin
			if (bird_x == 8'd0) begin
				bird_colour = 3'b001;
			end
			else if (bird_x == 8'd1 || bird_x == 8'd5) begin
				bird_colour = 3'b011;
			end
			else if (bird_x == 8'd6) begin
				bird_colour = 3'b100;
			end
            else begin 
                bird_colour = 3'b111;
            end
		end
    end

	// Move Bird_x
	reg [7:0] bird_move_x;
	// The move counter for Tree
	always @(posedge clock)
	begin : move_bird_counter_x
		if (!resetn || control_reset) begin
			bird_move_x <= 8'd140;
		end
		else if (move) begin
			if (bird_move_x == 8'd0) begin
				if (LFSR_out == 2'b00) begin
					bird_move_x <= 8'd170;
				end
				else if (LFSR_out == 2'b01) begin
					bird_move_x <= 8'd190;
				end
				else if (LFSR_out == 2'b10) begin
					bird_move_x <= 8'd210;
				end
				else if (LFSR_out == 2'b11) begin
					bird_move_x <= 8'd230;
				end
			end
			else begin
				bird_move_x <= bird_move_x - 2'd1;
			end
		end
	end


	// Move Tree_x
	reg [7:0] tree_move_x;
	// The move counter for Tree
	always @(posedge clock)
	begin : move_tree_counter_x
		if (!resetn || control_reset) begin
			tree_move_x <= 8'd100;
		end
		else if (move) begin
			if (tree_move_x == 8'd0) begin
				if (LFSR_out == 2'b00) begin
					tree_move_x <= 8'd225;
				end
				else if (LFSR_out == 2'b01) begin
					tree_move_x <= 8'd195;
				end
				else if (LFSR_out == 2'b10) begin
					tree_move_x <= 8'd180;
				end
				else if (LFSR_out == 2'b11) begin
					tree_move_x <= 8'd200;
				end
			end
			else begin
				tree_move_x <= tree_move_x - 2'd1;
			end
		end
	end


	// Move Cloud_x
	reg [7:0] cloud_move_x;
	// The move counter for Cloud
	always @(posedge clock)
	begin : move_cloud_counter_x
		if (!resetn || control_reset) begin
			cloud_move_x <= 8'd210;
		end
		else if (move) begin
			if (cloud_move_x == 8'd0) begin
				cloud_move_x <= 8'd210;
			end
			else begin
				cloud_move_x <= cloud_move_x - 1'b1;
			end
		end
	end


	// Move Bird_y
	reg [6:0] bird_move_y;
	// The move counter for Tree
	always @(posedge clock)
	begin : move_bird_counter_y
		if (!resetn || control_reset) begin
			bird_move_y <= 7'd103;
		end
		else if (bird_move_x == 8'd165) begin
			if (LFSR_out == 2'b00) begin
				bird_move_y <= 7'd84;
			end
			else if (LFSR_out == 2'b01) begin
				bird_move_y <= 7'd109;
			end
			else if (LFSR_out == 2'b10) begin
				bird_move_y <= 7'd97;
			end
			else if (LFSR_out == 2'b11) begin
				bird_move_y <= 7'd105;
			end
		end
	end


	// Move Tree_y
	reg [6:0] tree_move_y;
	// The move counter for Tree
	always @(posedge clock)
	begin : move_tree_counter_y
		if (!resetn || control_reset) begin
			tree_move_y <= 7'd104;
		end
		else if (tree_move_x == 8'd165) begin
			if (LFSR_out == 2'b00) begin
				tree_move_y <= 7'd112;
			end
			else if (LFSR_out == 2'b01) begin
				tree_move_y <= 7'd98;
			end
			else if (LFSR_out == 2'b10) begin
				tree_move_y <= 7'd100;
			end
			else if (LFSR_out == 2'b11) begin
				tree_move_y <= 7'd105;
			end
		end
	end


	// Move Pikachu
	reg [7:0] pikachu_move_x;
    reg [6:0] pikachu_move_y;
    // The move counter for Pikachu
    always @(posedge clock)
    begin : move_pikachu_counter
        if (!resetn || control_reset) begin
            pikachu_move_y <= 7'd108;
			pikachu_move_x <= 8'd10;
        end
        else if (move) begin
			if (is_jump) begin
				if (down == 1'b1) 
					pikachu_move_y <= pikachu_move_y + 1'b1;
            	else 
					pikachu_move_y <= pikachu_move_y - 1'b1;
			end
        end
    end

	// Move Dead Pikachu
	reg [7:0] dead_move_x;
    reg [6:0] dead_move_y;
    // The move counter for Dead Pikachu
    always @(posedge clock)
    begin : move_dead_counter
		if (!resetn) begin
			dead_move_x <= 8'd0;
			dead_move_y <= 7'd0;
		end
        else if (draw_death) begin
			if (change_status) begin
				dead_move_x <= pikachu_move_x + 1'b1;
			end
			else begin
				dead_move_x <= pikachu_move_x;
			end
			dead_move_y <= pikachu_move_y;
        end
        else if (move_death) begin
			dead_move_y <= dead_move_y  + 1'b1;
        end
    end


	// delay counter
	reg [25:0] count;
    wire en_frame;

	always @(posedge clock)
	begin : delay_counter
		if (!resetn || delay_reset)
			count <= 26'd499999;
        else if (en_delay) begin
			if (count == 26'd0) count <= 26'd499999;
		    else begin
				count <= count - 1'b1; 
			end
        end
	end

	assign en_frame = (count == 26'd0) ? 1'b1 : 1'b0;
	// frame counter
    reg [3:0] frame;

	always @(posedge clock)
	begin: frame_counter
		if (!resetn || delay_reset) frame <= 4'd0;
		else if (en_frame == 1'b1) begin
			if (frame == 4'd2) frame <= 4'd0;
			else begin
				frame <= frame + 1'b1;
			end
		end
	end

	assign stop = (frame == 4'd2) ? 1'b1 : 1'b0;

    reg [27:0] count_s;
	reg [27:0] count_e;
	always @(posedge clock)
	begin : start_delay_counter
		if (!resetn || delay_reset) begin
			count_s <= 28'd4_999_999;
			count_e <= 28'd19_999_999;
		end
        else if (start_en_delay) begin
			if (count_s == 28'd0) begin
				count_s <= 28'd4_999_999;
			end
		    else begin
				count_s <= count_s - 1'b1;
			end
        end
		else if (erase_en_delay) begin
			if (count_e == 28'd0) begin
				count_e <= 28'd19_999_999;
			end
		    else begin
				count_e <= count_e - 1'b1;
			end
		end
	end

	assign finish_change_letter = (count_s == 28'd0) ? 1'b1 : 1'b0;
	assign start_e = (count_e == 28'd0) ? 1'b1 : 1'b0;

	//random height
	wire [1:0] LFSR_out;
	LFSR l0(.clock(clock), 
			.resetn(resetn), 
			.control_reset(control_reset),
			.out(LFSR_out));
	

	reg [27:0] count_hold;
	reg [5:0] count_clear;

	always @(posedge clock)
	begin : rateDivider
		if (!resetn || control_reset) begin
			count_hold <= 28'd9_999_999;
		end
        else if (change_status) begin
			if (count_hold == 28'd0) begin
				count_hold <= 28'd9_999_999;
			end
		    else begin
				count_hold <= count_hold - 1'b1; 
			end

        end
	end

	assign clear = down ? 1'b1 : 1'b0;
	assign start_holding = (count_hold == 28'd0) ? 1'b1 : 1'b0;

	wire start_holding, clear;

	always @(posedge clock)
    begin
        if (!resetn || control_reset || clear)
            holding <= 5'd0;
        else if (start_holding == 1'b1) begin
			if (holding == 5'd9)
                holding <= 5'd0;
           	else
                holding <= holding + 1'b1;
    	end
    end
endmodule

module LFSR(clock, resetn, control_reset, out);
	input clock;
	input resetn;
	input control_reset;
	output [1:0] out;
	
	wire [15:0] initial_state;
	assign initial_state = 16'b1010110011100001;
	reg [15:0] lfsr;
	wire xor_out;
	assign xor_out = (((lfsr[0] ^ lfsr[2]) ^ lfsr[3]) ^ lfsr[5]) + 1;

	always @(posedge clock) begin
		if (!resetn || control_reset) begin
			lfsr <= initial_state;
		end
		else begin
			lfsr <= {xor_out, lfsr[15:1]};
		end
	end
	
	assign out = lfsr[15:14];
endmodule
