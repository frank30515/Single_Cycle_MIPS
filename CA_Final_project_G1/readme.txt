����Ƨ��D�n�]�t��ӳ���baseline�PExtension

/baseline
  /rtl
  �b����Ƨ���CHIP.v�A�ΨӴ���baseline��testbench

  /syn
  �b����Ƨ��]�t�Ҧ�CHIP.v���X����

/Extension
  ����Ƨ��]�t�t�~��Ӹ�Ƨ��A���O��BrPred�ML2cache
  / BrPred
      /rtl
      �ڭ��`�@���F3�ؤ��P��Branch prediction unit�A���O��1 bit�B2 bits�Mtwo
      level adaptive�A��.v�ɤ��O��CHIP_1bit.v�BCHIP_2bit.v�MCHIP_PHT2.v

      /syn
      �b����Ƨ��]�tCHIP_1bit.v�BCHIP_2bit.v�MCHIP_PHT2.v���X����

  / L2cache
      /rtl
      �b����Ƨ���5�ؤ��P���ɮסACHIP.v�BFinal_tb.v�BL1cache.v�BL2_DCache.v�ML2_ICache.v

      CHIP.v
      ���ɮ׻Pbaseline����CHIP.v�ۦ��A���t�~�[�F�X��port�A���F�[���hit/miss rate

      Final_tb.v
      ���ɮ׻P�U�е������ɬۦ��A���h�[�F�X��port�A���F�[���hit/miss rate

      L1cache.v
      ���ɮץΨӳ]�pMIPS����L1 cache�A���ǥѦbcommand line�Wdefine���P���ȡA�i�H���ͥX�j�p
      ��16 words/32 words�Mdirect mapped/2 way��L1 cache
      �Ҧp +define+L1SIZE_8+L1_2way�A�Y��L1��32words(8 blocks)�B�O2 way��cache

      L2_DCache.v�ML2_ICache.v
      ������ɮ׬ҬO�Ψӳ]�pMIPS����L2 cache�A���ǥѦbcommand line�Wdefine���P���ȡA�i�H��
      �ͥX�j�p��64 words/128 words/256 words�Mdirect mapped/2 way/4 way/8 way��L2 cache
      �Ҧp +define+L2_D_2way+DIRECTMAPI+L2SIZE_64�A�Y��D cache��2 way�AI cache��direct mapped
           �B���size���O64 words��cache