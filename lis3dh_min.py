class LIS3DH:
    def __init__(self, i2c, addr=0x18):
        self.i2c = i2c
        self.addr = addr
        if i2c.readfrom_mem(addr, 0x0F, 1)[0] != 0x33:
            raise OSError("LIS3DH not found (check wiring/address)")
        i2c.writeto_mem(addr, 0x20, b'\x57')   # 100 Hz, xyz on
        i2c.writeto_mem(addr, 0x23, b'\x88')   # high-res, ±2g

    def read_g(self):
        d = self.i2c.readfrom_mem(self.addr, 0x28 | 0x80, 6)
        out = []
        for i in range(0, 6, 2):
            raw = d[i] | (d[i + 1] << 8)
            if raw > 32767:
                raw -= 65536
            out.append((raw >> 4) / 1000)
        return out   # [x, y, z] in g
    
