/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class actuator_response_t implements lcm.lcm.LCMEncodable
{
    public long id;
    public double position;
    public double velocity;
    public double current;
 
    public actuator_response_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x5506640baf43a4c5L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.actuator_response_t.class))
            return 0L;
 
        classes.add(lcmtypes.actuator_response_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.id); 
 
        outs.writeDouble(this.position); 
 
        outs.writeDouble(this.velocity); 
 
        outs.writeDouble(this.current); 
 
    }
 
    public actuator_response_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public actuator_response_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.actuator_response_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.actuator_response_t o = new lcmtypes.actuator_response_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.id = ins.readLong();
 
        this.position = ins.readDouble();
 
        this.velocity = ins.readDouble();
 
        this.current = ins.readDouble();
 
    }
 
    public lcmtypes.actuator_response_t copy()
    {
        lcmtypes.actuator_response_t outobj = new lcmtypes.actuator_response_t();
        outobj.id = this.id;
 
        outobj.position = this.position;
 
        outobj.velocity = this.velocity;
 
        outobj.current = this.current;
 
        return outobj;
    }
 
}

