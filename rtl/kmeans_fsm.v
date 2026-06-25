// =============================================================
//  kmeans_fsm.v  –  K-Means Control Finite State Machine
//
//  States:
//    IDLE   : Wait for start pulse
//    INIT   : Latch points[0] & points[1] as initial centroids
//    ASSIGN : Assert assign_en; wait 1 cycle for combinational PEs
//    UPDATE : Assert update_en; wait for centroid_updater done pulse
//    CHECK  : Compare new vs old centroids; iterate or finish
//    DONE   : Hold final centroids, assert done flag
//
//  Target : xc7z020clg400-1 (Zynq-7000)
// =============================================================
`timescale 1ns / 1ps

module kmeans_fsm #(
    parameter DATA_W   = 8,
    parameter MAX_ITER = 15     // stop after this many iterations
)(
    input            clk,
    input            rst_n,
    input            start,

    // Centroid comparison inputs (from top-level registers)
    input [DATA_W-1:0] cur_cx0, cur_cy0,
    input [DATA_W-1:0] cur_cx1, cur_cy1,
    input [DATA_W-1:0] new_cx0, new_cy0,
    input [DATA_W-1:0] new_cx1, new_cy1,

    // Signal from centroid_updater
    input            updater_done,

    // Control outputs
    output reg       assign_en,  // enable PE assign phase (1 cycle)
    output reg       update_en,  // trigger centroid_updater
    output reg       latch_init, // latch initial centroids from points[0,1]
    output reg       latch_new,  // latch new centroids from updater
    output reg       done,       // clustering complete

    // Diagnostics
    output reg [3:0] iter_count, // iteration counter
    output reg [2:0] state_dbg   // current FSM state for waveform debug
);

    // --------------------------------------------------------
    // State encoding
    // --------------------------------------------------------
    localparam ST_IDLE   = 3'd0;
    localparam ST_INIT   = 3'd1;
    localparam ST_ASSIGN = 3'd2;
    localparam ST_UPDATE = 3'd3;
    localparam ST_CHECK  = 3'd4;
    localparam ST_DONE   = 3'd5;

    reg [2:0] state, next_state;

    // --------------------------------------------------------
    // Convergence check (combinational)
    // --------------------------------------------------------
    wire centroids_stable;
    assign centroids_stable = (new_cx0 == cur_cx0) &&
                              (new_cy0 == cur_cy0) &&
                              (new_cx1 == cur_cx1) &&
                              (new_cy1 == cur_cy1);

    // --------------------------------------------------------
    // State register
    // --------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= ST_IDLE;
        else
            state <= next_state;
    end

    // --------------------------------------------------------
    // Next-state logic (combinational)
    // --------------------------------------------------------
    always @(*) begin
        next_state = state; // default: hold
        case (state)
            ST_IDLE   : if (start)            next_state = ST_INIT;
            ST_INIT   :                        next_state = ST_ASSIGN;
            ST_ASSIGN :                        next_state = ST_UPDATE;
            ST_UPDATE : if (updater_done)      next_state = ST_CHECK;
            ST_CHECK  : begin
                if (centroids_stable || iter_count >= MAX_ITER[3:0])
                                               next_state = ST_DONE;
                else                           next_state = ST_ASSIGN;
            end
            ST_DONE   : if (start)             next_state = ST_IDLE; // re-run
            default   :                        next_state = ST_IDLE;
        endcase
    end

    // --------------------------------------------------------
    // Output logic + iteration counter
    // --------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            assign_en  <= 1'b0;
            update_en  <= 1'b0;
            latch_init <= 1'b0;
            latch_new  <= 1'b0;
            done       <= 1'b0;
            iter_count <= 4'd0;
            state_dbg  <= ST_IDLE;
        end else begin
            // Default: de-assert all pulses
            assign_en  <= 1'b0;
            update_en  <= 1'b0;
            latch_init <= 1'b0;
            latch_new  <= 1'b0;
            done       <= 1'b0;
            state_dbg  <= state;

            case (state)
                ST_IDLE: begin
                    iter_count <= 4'd0;
                end

                ST_INIT: begin
                    latch_init <= 1'b1;   // load points[0] & points[1] as centroids
                    iter_count <= 4'd0;
                end

                ST_ASSIGN: begin
                    assign_en  <= 1'b1;   // PEs compute cluster assignments
                end

                ST_UPDATE: begin
                    if (!updater_done)
                        update_en <= 1'b1; // trigger centroid_updater
                end

                ST_CHECK: begin
                    if (!centroids_stable && iter_count < MAX_ITER[3:0]) begin
                        latch_new  <= 1'b1;             // accept new centroids
                        iter_count <= iter_count + 4'd1;
                    end
                end

                ST_DONE: begin
                    done <= 1'b1;
                end

                default: ; // nothing
            endcase
        end
    end

endmodule
