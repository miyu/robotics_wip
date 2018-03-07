#pragma once

#include "Networking.h"
#include <string>

typedef TSharedPtr<FInternetAddr> PRemoteAddr;

DECLARE_DELEGATE_TwoParams(FUdpDataReceived, const uint8*, const uint32)


class UdpClient : public TSharedFromThis<UdpClient>
{
   PRemoteAddr remoteAddr;
   FSocket *socket = nullptr;
   FUdpSocketReceiver * receiver = nullptr;
   FUdpDataReceived onDataReceived;

public:
   UdpClient(PRemoteAddr remoteAddr);
   ~UdpClient();

   void Send(const uint8 *data, int32 count);
   FUdpDataReceived& OnDataReceived();
   void Shutdown();

   void HandleDataReceivedInternal(const FArrayReaderPtr& reader, const FIPv4Endpoint& remote);

public:
   static PRemoteAddr CreateUdpEndpoint(std::wstring host, uint32 port);
};