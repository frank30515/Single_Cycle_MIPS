此資料夾主要包含兩個部分baseline與Extension

/baseline
  /rtl
  在此資料夾有CHIP.v，用來測試baseline的testbench

  /syn
  在此資料夾包含所有CHIP.v的合成檔

/Extension
  此資料夾包含另外兩個資料夾，分別為BrPred和L2cache
  / BrPred
      /rtl
      我們總共做了3種不同的Branch prediction unit，分別為1 bit、2 bits和two
      level adaptive，其.v檔分別為CHIP_1bit.v、CHIP_2bit.v和CHIP_PHT2.v

      /syn
      在此資料夾包含CHIP_1bit.v、CHIP_2bit.v和CHIP_PHT2.v的合成檔

  / L2cache
      /rtl
      在此資料夾有5種不同的檔案，CHIP.v、Final_tb.v、L1cache.v、L2_DCache.v和L2_ICache.v

      CHIP.v
      此檔案與baseline中的CHIP.v相似，但另外加了幾個port，為了觀察其hit/miss rate

      Final_tb.v
      此檔案與助教給的原檔相似，但多加了幾個port，為了觀察其hit/miss rate

      L1cache.v
      此檔案用來設計MIPS當中的L1 cache，而藉由在command line上define不同的值，可以產生出大小
      為16 words/32 words和direct mapped/2 way的L1 cache
      例如 +define+L1SIZE_8+L1_2way，即為L1為32words(8 blocks)且是2 way的cache

      L2_DCache.v和L2_ICache.v
      此兩個檔案皆是用來設計MIPS當中的L2 cache，而藉由在command line上define不同的值，可以產
      生出大小為64 words/128 words/256 words和direct mapped/2 way/4 way/8 way的L2 cache
      例如 +define+L2_D_2way+DIRECTMAPI+L2SIZE_64，即為D cache為2 way，I cache為direct mapped
           且兩者size都是64 words的cache