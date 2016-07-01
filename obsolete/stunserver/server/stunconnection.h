#ifndef STUN_CONNECTION_H
#define STUN_CONNECTION_H

#include "stuncore.h"
#include "stunsocket.h"


enum StunConnectionState
{
    ConnectionState_Idle,
    ConnectionState_Receiving,
    ConnectionState_Transmitting,
    ConnectionState_Closing,     // shutdown has been called, waiting for close notification on other end
};


struct StunConnection
{
    time_t _timeStart;
    StunConnectionState _state;
    CStunSocket _stunsocket;
    CStunMessageReader _reader;
    CRefCountedBuffer _spReaderBuffer;
    CRefCountedBuffer _spOutputBuffer;  // contains the response
    size_t _txCount;     // number of bytes of response transmitted thus far
    int _idHashTable; // hints at which hash table the connection got inserted into
    StunConnection* pNext; // next item in pool - meaningless outside of the pool
};



class CConnectionPool
{
private:

    StunConnection* _freelist;
    
    HRESULT Grow();
    
   
public:
    CConnectionPool();
    ~CConnectionPool();
    
    StunConnection* GetConnection(int sock, SocketRole role);
    void ReleaseConnection(StunConnection* pConn);
    
    void Reset();
    
};






#endif

