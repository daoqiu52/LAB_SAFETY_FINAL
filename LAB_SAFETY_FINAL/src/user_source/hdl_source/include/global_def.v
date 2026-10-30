


`define   DATA_WIDTH                        32
`define   ADDR_WIDTH                        21
`define   DM_WIDTH                          4

`define   ROW_WIDTH                        11
`define   BA_WIDTH                        2

// Parenthesize macros: textual expansion must preserve the clock period.
`define   SDR_CLK_PERIOD                    (1000000000 / 50000000)
// The controller counts from zero through this limit (inclusive).
// Subtract one so the full row sweep stays within the 64 ms budget.
`define   SELF_REFRESH_INTERVAL             ((64000000 / `SDR_CLK_PERIOD / (2 ** (`ROW_WIDTH))) - 1)

