#include "can_wrap.hpp"
#include <uv.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <node.h>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdlib>

using Nan::Callback;
using v8::Local;
using v8::Function;
using v8::FunctionTemplate;
using v8::Value;
using std::begin;
using std::copy_n;

namespace rawcan
{
Nan::Persistent<Function> CANWrap::s_constructor;

NAN_MODULE_INIT(CANWrap::Initialize)
{
    Local<FunctionTemplate> tpl = Nan::New<FunctionTemplate>(New);
    tpl->SetClassName(Nan::New("CANWrap").ToLocalChecked());
    tpl->InstanceTemplate()->SetInternalFieldCount(1);

    SetPrototypeMethod(tpl, "bind", Bind);
    SetPrototypeMethod(tpl, "send", Send);
    SetPrototypeMethod(tpl, "close", Close);
    SetPrototypeMethod(tpl, "setFilter", SetFilter);
    SetPrototypeMethod(tpl, "onSent", OnSent);
    SetPrototypeMethod(tpl, "onMessage", OnMessage);
    SetPrototypeMethod(tpl, "onError", OnError);
    SetPrototypeMethod(tpl, "ref", UvRef);
    SetPrototypeMethod(tpl, "unref", UvUnRef);

    s_constructor.Reset(Nan::GetFunction(tpl).ToLocalChecked());
    Nan::Set(target, Nan::New("CANWrap").ToLocalChecked(),
             Nan::GetFunction(tpl).ToLocalChecked());
}

CANWrap::CANWrap()
    : m_socket(socket(PF_CAN, SOCK_RAW, CAN_RAW))
{
    assert(m_socket);
    // set nonblocking mode
    int flags = fcntl(m_socket, F_GETFL, 0);
    fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);

    uv_poll_init_socket(uv_default_loop(), &m_uvHandle, m_socket);
    m_uvHandle.data = this;
}

NAN_METHOD(CANWrap::New)
{
    assert(info.IsConstructCall());
    auto* can = new CANWrap();
    can->Wrap(info.This());
    info.GetReturnValue().Set(info.This());
}

NAN_METHOD(CANWrap::Bind)
{
    assert(1 == info.Length());
    CANWrap* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());

    assert(self);
    assert(!self->m_closed);

    Nan::Utf8String iface(info[0]->ToString());

    auto ifr = ifreq();
    strcpy(ifr.ifr_name, *iface);
    auto err = ioctl(self->m_socket, SIOCGIFINDEX, &ifr);

    if (err == 0)
    {
        auto canAddr = sockaddr_can();
        canAddr.can_family = AF_CAN;
        canAddr.can_ifindex = ifr.ifr_ifindex;

        err = bind(self->m_socket, reinterpret_cast<struct sockaddr*>(&canAddr),
                   sizeof(canAddr));
    }

    self->Ref();

    self->m_pollEvents |= UV_READABLE;
    self->doPoll();

    info.GetReturnValue().Set(err);
}

NAN_METHOD(CANWrap::Send)
{
    // send(id, buffer)
    assert(2 == info.Length());
    assert(info[0]->IsUint32());
    assert(node::Buffer::HasInstance(info[1]));

    auto* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());
    assert(self);
    assert(!self->m_closed);

    auto& sendBuffer = self->m_sendBuffer;
    auto id = Nan::To<uint32_t>(info[0]).FromJust();
    sendBuffer.can_id = id;

    auto length = node::Buffer::Length(info[1]);
    sendBuffer.can_dlc = length;
    copy_n(node::Buffer::Data(info[1]), length, begin(sendBuffer.data));

    self->m_pollEvents |= UV_WRITABLE;
    self->doPoll();
}

NAN_METHOD(CANWrap::Close)
{
    auto* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());
    assert(self);

    self->doClose();
}

NAN_METHOD(CANWrap::SetFilter)
{
    // setFilter(filter, mask)
    assert(2 == info.Length());
    assert(info[0]->IsUint32());
    assert(info[1]->IsUint32());

    auto filter = Nan::To<uint32_t>(info[0]).FromJust();
    auto mask = Nan::To<uint32_t>(info[1]).FromJust();

    auto* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());
    assert(self);

    struct can_filter cf[1];
    cf[0].can_id = filter;
    cf[0].can_mask = mask;
    setsockopt(self->m_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &cf, sizeof(cf));
}

NAN_METHOD(CANWrap::OnSent)
{
    // onSent(callback)
    assert(1 == info.Length());
    assert(info[0]->IsFunction());

    auto* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());
    assert(self);

    self->m_sentCallback.SetFunction(info[0].As<v8::Function>());
}

NAN_METHOD(CANWrap::OnMessage)
{
    // onMessage(callback)
    assert(1 == info.Length());
    assert(info[0]->IsFunction());

    auto* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());
    assert(self);

    self->m_messageCallback.SetFunction(info[0].As<v8::Function>());
}

NAN_METHOD(CANWrap::OnError)
{
    // onMessage(callback)
    assert(1 == info.Length());
    assert(info[0]->IsFunction());

    auto* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());
    assert(self);

    self->m_errorCallback.SetFunction(info[0].As<v8::Function>());
}

NAN_METHOD(CANWrap::UvRef)
{
    auto* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());
    assert(self);

    uv_ref(reinterpret_cast<uv_handle_t*>(&self->m_uvHandle));
}

NAN_METHOD(CANWrap::UvUnRef)
{
    auto* self = ObjectWrap::Unwrap<CANWrap>(info.Holder());
    assert(self);

    uv_unref(reinterpret_cast<uv_handle_t*>(&self->m_uvHandle));
}

void CANWrap::uvPollCallback(uv_poll_t* pollHandle, int status,
                             int events)
{
    auto* self = static_cast<CANWrap*>(pollHandle->data);
    assert(self);
    self->pollCallback(status, events);
}



// https://gcc.gnu.org/onlinedocs/gcc/Other-Builtins.html
// Branch prediction stuff... some optimization.
#define likely(x)   __builtin_expect( x , 1)

// symbols
#define SYMBOL(aString) Nan::New((aString)).ToLocalChecked()
#define tssec_symbol    SYMBOL("ts_sec")
#define tsusec_symbol   SYMBOL("ts_usec")
#define id_symbol       SYMBOL("id")
#define mask_symbol     SYMBOL("mask")
#define invert_symbol   SYMBOL("invert")
#define ext_symbol      SYMBOL("ext")
#define rtr_symbol      SYMBOL("rtr")
#define err_symbol      SYMBOL("err")
#define data_symbol     SYMBOL("data")

  // The code inside this function
  // was copied and slightly modified from 
  // the code in 'void async_receiver_ready(int status)' in the
  // node-can library: https://github.com/sebi2k1/node-can
  v8::Local<v8::Object> wrapMessage(int socket_filedesc,
                                    const can_frame& frame) {
    v8::Local<v8::Object> obj = Nan::New<v8::Object>();
    canid_t id = frame.can_id;
    bool isEff = frame.can_id & CAN_EFF_FLAG;
    bool isRtr = frame.can_id & CAN_RTR_FLAG;
    bool isErr = frame.can_id & CAN_ERR_FLAG;

    id = isEff ? frame.can_id & CAN_EFF_MASK 
      : frame.can_id & CAN_SFF_MASK;

    bool m_TimestampsSupported = true; // User parameter
    if (m_TimestampsSupported) {
      struct timeval tv;

      if (likely(ioctl(socket_filedesc, SIOCGSTAMP, &tv) >= 0)) {
        Nan::Set(obj, tssec_symbol, Nan::New((int32_t)tv.tv_sec));
        Nan::Set(obj, tsusec_symbol, Nan::New((int32_t)tv.tv_usec));
      }
    }
    Nan::Set(obj, id_symbol, Nan::New(id));

    if (isEff) {
      Nan::Set(obj, ext_symbol, Nan::New(isEff));
    }
    if (isRtr) {
      Nan::Set(obj, rtr_symbol, Nan::New(isRtr));
    }
    if (isErr) {
      Nan::Set(obj, err_symbol, Nan::New(isErr));
    }
    Nan::Set(obj, data_symbol, 
             Nan::CopyBuffer((char *)frame.data, 
                             frame.can_dlc & 0xf).ToLocalChecked());
    return obj;
  }

void CANWrap::pollCallback(int status, int events)
{
    if (status == 0)
    {
        if (events & UV_WRITABLE)
        {
            const int err = doSend() < 0 ? errno : 0;

            m_pollEvents &= ~UV_WRITABLE;
            doPoll();
            if (!m_sentCallback.IsEmpty())
            {
                Nan::HandleScope scope;
                Local<Value> argv[1] = {Nan::New(err)};
                m_sentCallback.Call(1, argv);
            }
            else
            {
                callErrorCallback(err);
            }
        }
        else if (events & UV_READABLE)
        {
            const int err = doRecv();
            if (err < 0)
            {
                callErrorCallback(errno);
            }
            else if (!m_messageCallback.IsEmpty())
            {   
              // Use the calling style of the node-can library.
              const bool use_node_can_style = true;

              if (use_node_can_style) {
                const int arg_count = 1;
                Nan::HandleScope scope;
                v8::Local<v8::Value> argv[arg_count] = {
                  wrapMessage(m_socket, m_recvBuffer)
                };
                m_messageCallback.Call(arg_count, argv);
              } else {
                Nan::HandleScope scope;
                Local<Value> argv[] = {
                    Nan::New(m_recvBuffer.can_id),
                    Nan::CopyBuffer(reinterpret_cast<char*>(&m_recvBuffer.data),
                                    m_recvBuffer.can_dlc)
                        .ToLocalChecked()};

                //// This is where we call the callback that we registered
                m_messageCallback.Call(1, argv);
              }
            }
        }
    }
    else
    {
        callErrorCallback(status);
    }
}

int CANWrap::doPoll()
{
    if (m_closed)
    {
        return -1;
    }

    if (m_pollEvents)
    {
        return uv_poll_start(&m_uvHandle, m_pollEvents, uvPollCallback);
    }
    else
    {
        return uv_poll_stop(&m_uvHandle);
    }
}

int CANWrap::doSend()
{
    return ::send(m_socket, &m_sendBuffer, sizeof(m_sendBuffer), 0);
}

int CANWrap::doRecv()
{
    return ::recv(m_socket, &m_recvBuffer, sizeof(m_recvBuffer), 0);
}

void CANWrap::doClose()
{
    if (!m_closed)
    {
        uv_poll_stop(&m_uvHandle);
        uv_close(reinterpret_cast<uv_handle_t*>(&m_uvHandle),
                 [](uv_handle_t* handle) {
                     auto* self = reinterpret_cast<CANWrap*>(handle->data);
                     assert(!self->persistent().IsEmpty());
                     self->Unref();
                 });
       m_closed = true;
    }
}

void CANWrap::callErrorCallback(int err)
{
    if (!m_errorCallback.IsEmpty())
    {
        Nan::HandleScope scope;
        Local<Value> argv[1] = {Nan::New(err)};
        m_errorCallback.Call(1, argv);
    }
}
}
