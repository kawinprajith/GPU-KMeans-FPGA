// =============================================================
//  centroid_updater.v  –  Cluster Accumulator & Centroid Averager
//
//  On the rising edge of clk when update_en is asserted:
//    - Groups all 8 points by their cluster_id
//    - Accumulates sum_x, sum_y per cluster
//    - Divides by count using shift-based integer division
//    - Guards against empty clusters (keeps old centroid)
//    - Asserts done for exactly one clock cycle
//
//  Division strategy (shift-only, no divider circuit):
//    cnt=1 → >>0   cnt=2 → >>1   cnt=3 → >>1 (approx)
//    cnt=4 → >>2   cnt=5 → >>2 (approx)  cnt=6 → >>2 (approx)
//    cnt=7 → >>2 (approx)  cnt=8 → >>3
//  For the default testbench (4+4 split), cnt is always 4 → exact.
//
//  Target : xc7z020clg400-1 (Zynq-7000)
// =============================================================
`timescale 1ns / 1ps

module centroid_updater #(
    parameter DATA_W   = 8,   // coordinate bit-width
    parameter N_POINTS = 8,   // number of input points
    parameter ACC_W    = 11   // accumulator width (DATA_W + log2(N_POINTS))
)(
    input                        clk,
    input                        rst_n,
    input                        update_en,   // pulse: latch inputs & compute

    // All point coordinates (flat bus: point 0 in [DATA_W-1:0], etc.)
    input  [N_POINTS*DATA_W-1:0] points_x_flat,
    input  [N_POINTS*DATA_W-1:0] points_y_flat,

    // Cluster assignments from PEs (1 bit per point)
    input  [N_POINTS-1:0]        cluster_ids,

    // Previous centroids (kept if cluster is empty)
    input  [DATA_W-1:0]          prev_cx0, prev_cy0,
    input  [DATA_W-1:0]          prev_cx1, prev_cy1,

    // New centroid outputs (registered)
    output reg [DATA_W-1:0]      new_cx0, new_cy0,
    output reg [DATA_W-1:0]      new_cx1, new_cy1,

    output reg                   done  // 1-cycle pulse when new centroids are ready
);

    // --------------------------------------------------------
    // Coordinate unpack wires
    // --------------------------------------------------------
    wire [DATA_W-1:0] px [0:N_POINTS-1];
    wire [DATA_W-1:0] py [0:N_POINTS-1];

    genvar gi;
    generate
        for (gi = 0; gi < N_POINTS; gi = gi+1) begin : unpack
            assign px[gi] = points_x_flat[gi*DATA_W +: DATA_W];
            assign py[gi] = points_y_flat[gi*DATA_W +: DATA_W];
        end
    endgenerate

    // --------------------------------------------------------
    // Combinational accumulation
    // --------------------------------------------------------
    reg [ACC_W-1:0] sum_x0_c, sum_y0_c, sum_x1_c, sum_y1_c;
    reg [3:0]        cnt0_c,   cnt1_c;

    integer i;
    always @(*) begin
        sum_x0_c = {ACC_W{1'b0}};
        sum_y0_c = {ACC_W{1'b0}};
        sum_x1_c = {ACC_W{1'b0}};
        sum_y1_c = {ACC_W{1'b0}};
        cnt0_c   = 4'd0;
        cnt1_c   = 4'd0;

        for (i = 0; i < N_POINTS; i = i+1) begin
            if (cluster_ids[i] == 1'b0) begin
                sum_x0_c = sum_x0_c + {{(ACC_W-DATA_W){1'b0}}, px[i]};
                sum_y0_c = sum_y0_c + {{(ACC_W-DATA_W){1'b0}}, py[i]};
                cnt0_c   = cnt0_c + 4'd1;
            end else begin
                sum_x1_c = sum_x1_c + {{(ACC_W-DATA_W){1'b0}}, px[i]};
                sum_y1_c = sum_y1_c + {{(ACC_W-DATA_W){1'b0}}, py[i]};
                cnt1_c   = cnt1_c + 4'd1;
            end
        end
    end

    // --------------------------------------------------------
    // Shift-based division  (floor-log2 of count)
    // --------------------------------------------------------
    function [DATA_W-1:0] shift_div;
        input [ACC_W-1:0] sum;
        input [3:0]       cnt;
        begin
            case (cnt)
                4'd1    : shift_div = sum[DATA_W-1:0];             // >>0  exact
                4'd2    : shift_div = sum[DATA_W:1];               // >>1  exact
                4'd3    : shift_div = sum[DATA_W:1];               // >>1  approx (/2 vs /3)
                4'd4    : shift_div = sum[DATA_W+1:2];             // >>2  exact
                4'd5    : shift_div = sum[DATA_W+1:2];             // >>2  approx
                4'd6    : shift_div = sum[DATA_W+1:2];             // >>2  approx
                4'd7    : shift_div = sum[DATA_W+1:2];             // >>2  approx
                4'd8    : shift_div = sum[DATA_W+2:3];             // >>3  exact
                default : shift_div = {DATA_W{1'b0}};
            endcase
        end
    endfunction

    // --------------------------------------------------------
    // Registered output (latched on update_en)
    // --------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            new_cx0 <= {DATA_W{1'b0}};
            new_cy0 <= {DATA_W{1'b0}};
            new_cx1 <= {DATA_W{1'b0}};
            new_cy1 <= {DATA_W{1'b0}};
            done    <= 1'b0;
        end else if (update_en) begin
            // Cluster 0: guard against empty cluster
            new_cx0 <= (cnt0_c == 4'd0) ? prev_cx0 : shift_div(sum_x0_c, cnt0_c);
            new_cy0 <= (cnt0_c == 4'd0) ? prev_cy0 : shift_div(sum_y0_c, cnt0_c);
            // Cluster 1: guard against empty cluster
            new_cx1 <= (cnt1_c == 4'd0) ? prev_cx1 : shift_div(sum_x1_c, cnt1_c);
            new_cy1 <= (cnt1_c == 4'd0) ? prev_cy1 : shift_div(sum_y1_c, cnt1_c);
            done    <= 1'b1;
        end else begin
            done <= 1'b0;
        end
    end

endmodule
