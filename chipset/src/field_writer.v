// Files active lines V_SKIP.. of each BT.656 field as {Y, Cb|Cr} into SDRAM row {slot, stored line} via a two-line buffer.
// Slot: the tag's phase (F before any tag); once tagged, only valid-tag fields are written, so a stalled deck keeps them.
module field_writer #(
    parameter integer V_SKIP   = 13,
    parameter integer LINES    = 240,
    parameter integer LAG      = 2,     // frame_start ends stored line LAG
    parameter integer TAG_LINE = V_SKIP - 3,
    parameter integer SCAN_HI  = V_SKIP + 7,    // search lines 0 to SCAN_HI too: pause and cue move the tag off TAG_LINE
    parameter integer COAST    = 4     // failed tags in a row filed by the predicted count before falling back to F
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  data,
    input  wire        raw,             // keep writing while frozen, so the raw view shows the input
    output reg         we,
    output reg  [10:0] waddr,
    output reg  [15:0] wdata,
    output reg         wr_req,          // pulse at the end of each stored line
    output reg  [12:0] wr_row,
    output reg         wr_half,
    output reg         line_start,      // pulse on every EAV
    output reg         frame_start,     // pulse on the EAV ending active line V_SKIP + LAG
    output wire        field_start,     // the SAV of stored line 0
    output reg         tag_strobe,      // pulse once per field, after its tag line
    output reg         tag_ok,          // with tag_strobe: this field's tag was valid
    output reg         tag_hit,         // pulse on the first valid tag of a field on lines 0 to SCAN_HI
    output reg  [8:0]  tag_hit_line,    // with tag_hit: its active line
    output reg  [7:0]  tag_hit_count,   // with tag_hit: its count
    output reg         vitc_hit,        // pulse on each line of 0 to SCAN_HI with a valid VITC word
    output reg  [8:0]  vitc_hit_line,
    output reg  [63:0] vitc_data,       // with vitc_hit: its data bits (vitc_slicer)
    output reg         ptag_hit,        // pulse per position tag read on lines 0 to SCAN_HI (ptag_slicer)
    output reg  [15:0] ptag_pos,
    output reg  [6:0]  ptag_at,
    output reg  [8:0]  ptag_line,
    output reg         vitc_line_done,  // pulse after each line of 0 to SCAN_HI, with vitc_hit_line and:
    output reg  [2:0]  vitc_why,        // vitc_slicer's verdict,
    output reg  [3:0]  vitc_why_grp,    // its first bad sync group,
    output reg         vitc_like,       // and whether the line starts like VITC
    output wire        tap_sav,         // every stored or searched line: SAV, each luma sample, the line
    output wire        tap_y_en,
    output wire [7:0]  tap_y,
    output wire [8:0]  tap_line,
    output wire        tag_held,        // with tag_strobe: this field is filed by tag, valid or predicted
    output reg  [3:0]  tag_scheme,      // from the last valid tag
    output reg  [7:0]  tag_count,
    output reg         frozen,          // tag lost past the coast after one was seen: slots hold their last fields
    output reg  [1:0]  buf_slot,        // slot the current field is being written to
    output wire        st_sav,          // stored line: SAV, each luma sample, EAV (audio_slicer)
    output wire        st_y_en,
    output wire [7:0]  st_y,
    output wire        st_eav,
    output wire [7:0]  st_row           // stored line within the field, with st_sav
);
    `include "sdram_map.vh"
    reg  [7:0] b0, b1, b2, b3;
    wire       is_code = b3 == 8'hFF && b2 == 8'h00 && b1 == 8'h00;
    wire       f = b0[6], v = b0[5], h = b0[4];
    wire       prot_ok = b0[7] && b0[3:0] == {v ^ h, f ^ h, f ^ v, f ^ v ^ h};

    reg        in_line;                 // inside a stored or tag line's active video
    reg        in_act;                  // inside any active line (the line_cap tap)
    reg        store;                   // ... and it is a stored line
    reg        wr_ok;                   // ... and goes to SDRAM
    reg [10:0] k;                       // byte index after SAV: even = Cb/Cr, odd = Y
    reg [8:0]  line;                    // active line within the field
    reg [7:0]  sl;                      // stored line within the field
    reg [7:0]  chroma;
    wire       stored = line >= V_SKIP && line < V_SKIP + LINES;
    wire       y_en   = in_line && k < 11'd1440 && k[0] && !is_code;

    wire       sav     = is_code && prot_ok && !v && !h;
    wire       eav     = is_code && prot_ok && !v && h;
    assign     st_sav  = sav && stored;
    assign     st_eav  = eav && stored;
    assign     st_y_en = y_en && store;
    assign     st_y    = b0;
    assign     st_row  = sl;
    assign     field_start = st_sav && sl == 8'd0;
    assign     tap_sav  = sav;
    assign     tap_y_en = in_act && k < 11'd1440 && k[0] && !is_code;
    assign     tap_y    = b0;
    assign     tap_line = line;
    reg        t_found;                 // this field's first valid tag has been read
    reg        t_open;                  // the slicer saw this line's SAV (a field's first EAV comes before any)
    wire       t_win   = line <= SCAN_HI;
    wire       t_start = sav && t_win && !t_found;
    wire       t_done  = eav && t_open;
    wire       t_ok;
    wire [3:0] t_scheme;
    wire [7:0] t_count;
    tag_slicer slicer (
        .clk(clk), .start(t_start), .y_en(y_en), .y(b0), .done(t_done),
        .ok(t_ok), .scheme(t_scheme), .count(t_count)
    );

    // Position tags: every line of the search window, any number per line.
    wire        p_hit;
    wire [15:0] p_pos;
    wire [6:0]  p_at;
    ptag_slicer pslicer (
        .clk(clk), .start(sav && t_win), .y_en(y_en && line <= SCAN_HI), .y(b0), .hit(p_hit), .pos(p_pos), .at(p_at)
    );
    always @(posedge clk) begin
        ptag_hit <= p_hit;
        if (p_hit) {ptag_pos, ptag_at, ptag_line} <= {p_pos, p_at, line};
    end

    // VITC: every line of the search window is read; each good one is reported.
    reg        v_open, v_chk;
    reg [8:0]  v_line;
    wire       v_ok, v_like;
    wire [63:0] v_data;
    wire [2:0] v_why;
    wire [3:0] v_why_grp;
    vitc_slicer vslicer (
        .clk(clk), .start(sav && t_win), .y_en(y_en), .y(b0), .done(eav && v_open), .ok(v_ok), .data(v_data),
        .why(v_why), .why_grp(v_why_grp), .like(v_like)
    );
    always @(posedge clk) begin
        v_chk    <= eav && v_open;
        vitc_hit <= v_chk && v_ok;
        vitc_line_done <= v_chk;
        if (v_chk) {vitc_why, vitc_why_grp, vitc_like} <= {v_why, v_why_grp, v_like};
        if (eav && v_open) v_line <= line;
        if (sav && t_win) v_open <= 1'b1;
        else if (is_code && prot_ok && h) v_open <= 1'b0;
        if (v_chk) begin
            vitc_hit_line <= v_line;
            vitc_data     <= v_data;
        end
    end

    // Tag scheme = {t_log2, layout}, count = {240p, count[6:0]}; a field's slot is its phase, count mod T.
    reg        t_chk, t_last, t_latch, field_ok, had_tag;
    assign     tag_held = field_ok;     // set at t_latch, one cycle before tag_strobe; cleared only at vertical blanking
    reg [8:0]  t_line;
    reg [3:0]  f_scheme;
    reg [7:0]  f_count;
    wire       t_play = t_found && tag_hit_line == TAG_LINE;
    reg [1:0]  field_slot;
    reg [6:0]  pred;                   // this field's count: the tag's, or the last one's + 1
    localparam [2:0] MISS_MAX = COAST;
    reg [2:0]  misses = MISS_MAX;       // failed tags in a row, saturating at COAST
    wire [6:0] p_next = pred + 1'b1;
    wire       coast  = misses < MISS_MAX;
    always @(posedge clk) begin
        t_chk      <= t_done;
        t_last     <= eav && line == TAG_LINE;
        t_latch    <= t_last;
        tag_strobe <= t_latch;
        tag_hit    <= 1'b0;
        if (t_done) t_line <= line;
        if (t_start) t_open <= 1'b1;
        else if (is_code && prot_ok && h) t_open <= 1'b0;
        if (is_code && prot_ok && v) t_found <= 1'b0;
        else if (t_chk && t_ok && !t_found) begin
            t_found      <= 1'b1;
            f_scheme     <= t_scheme;
            f_count      <= t_count;
            tag_hit      <= 1'b1;
            tag_hit_line <= t_line;
            tag_hit_count <= t_count;
        end
        if (rst) begin
            misses <= MISS_MAX;
            had_tag <= 1'b0;
            frozen <= 1'b0;
        end else if (t_latch) begin
            tag_ok <= t_play;
            frozen <= had_tag && !t_play && !coast;
            if (t_play) begin
                had_tag    <= 1'b1;
                tag_scheme <= f_scheme;
                tag_count  <= f_count;
                pred       <= f_count[6:0];
                misses     <= 3'd0;
                field_ok   <= 1'b1;
                field_slot <= f_count[1:0] & {f_scheme[3], |f_scheme[3:2]};
            end else begin
                pred       <= p_next;
                misses     <= misses + (misses != MISS_MAX);
                field_ok   <= coast;
                field_slot <= p_next[1:0] & {tag_scheme[3], |tag_scheme[3:2]};
            end
        end
        if (is_code && prot_ok && v) field_ok <= 1'b0;
    end

    always @(posedge clk) begin
        {b3, b2, b1, b0} <= {b2, b1, b0, data};
        we          <= 1'b0;
        wr_req      <= 1'b0;
        line_start  <= 1'b0;
        frame_start <= 1'b0;
        if (rst) begin
            in_line <= 1'b0;
            in_act  <= 1'b0;
            line    <= 9'd0;
            sl      <= 8'd0;
        end else if (is_code && prot_ok) begin
            if (h) line_start <= 1'b1;
            if (v) begin                // vertical blanking: next active line starts a new field
                in_line <= 1'b0;
                in_act  <= 1'b0;
                line    <= 9'd0;
                sl      <= 8'd0;
            end else if (!h) begin      // SAV
                in_line  <= stored || t_win;
                in_act   <= 1'b1;
                store    <= stored;
                wr_ok    <= tag_ok || !had_tag || raw;  // a coasted field may be a paused repeat of the other phase
                k        <= 11'd0;
                buf_slot <= field_ok ? field_slot : {1'b0, f};
            end else begin              // EAV
                in_line <= 1'b0;
                in_act  <= 1'b0;
                if (stored) begin
                    wr_req  <= wr_ok;
                    wr_row  <= video_row(buf_slot, sl);
                    wr_half <= sl[0];
                    sl      <= sl + 1'b1;
                end
                if (line == V_SKIP + LAG) frame_start <= 1'b1;
                line <= line + (line != 9'h1FF);
            end
        end else if (in_act && k < 11'd1440) begin
            k <= k + 1'b1;
            if (!k[0]) chroma <= b0;
            else if (store) begin
                we    <= wr_ok;
                waddr <= {sl[0], k[10:1]};
                wdata <= {b0, chroma};
            end
        end
    end
endmodule
