#include "UdpClient.hpp"

const int kUdpBufferSize = 2 * 1024 * 1024;

UdpClient::UdpClient(PRemoteAddr remoteAddr) : remoteAddr(remoteAddr)
{
   auto endpoint = CreateUdpEndpoint(L"0.0.0.0", 21337);
   socket = FUdpSocketBuilder(L"UdpClient socket").AsReusable()
                                                  .AsNonBlocking()
                                                  //.WithBroadcast()
                                                  //.WithSendBufferSize(kUdpBufferSize)
                                                  .WithReceiveBufferSize(kUdpBufferSize)
                                                  .BoundToEndpoint(endpoint)
                                                  .Build();

   if (!socket) {
      UE_LOG(LogTemp, Error, TEXT("UdpClient::UdpClient Failed to init broadcast socket"));
   } else {
      receiver = new FUdpSocketReceiver(socket, 0, TEXT("UdpClient receiver")); // todo: arg1 val?
      receiver->OnDataReceived().BindRaw(this, &UdpClient::HandleDataReceivedInternal);
      receiver->Start();
   }
}

UdpClient::~UdpClient()
{
   Shutdown();
}

void UdpClient::Send(const uint8 *data, int32 count)
{
   int bytesSent;
   socket->SendTo(data, count, bytesSent, *remoteAddr);
   check(bytesSent == count);
}

FUdpDataReceived& UdpClient::OnDataReceived()
{
   return onDataReceived;
}

void UdpClient::Shutdown()
{
   delete receiver;
   receiver = nullptr;

   if (socket)
   {
      socket->Close();
      ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(socket);
      socket = nullptr;
   }
}

void UdpClient::HandleDataReceivedInternal(const FArrayReaderPtr& reader, const FIPv4Endpoint& remote)
{
   onDataReceived.ExecuteIfBound(reader->GetData(), reader->Num());
}

PRemoteAddr UdpClient::CreateUdpEndpoint(std::wstring host, uint32 port)
{
   auto addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

   if (host == L"0.0.0.0") {
      addr->SetAnyAddress();
   } else if (host == L"255.255.255.255") {
      addr->SetBroadcastAddress();
   } else {
      bool bIsValid;
      addr->SetIp(host.c_str(), bIsValid);
      check(bIsValid);
   }
   addr->SetPort(port);

   return addr;
}
