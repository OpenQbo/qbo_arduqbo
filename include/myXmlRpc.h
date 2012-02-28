#ifndef MYXMLRPC_H
#define MYXMLRPC_H

namespace XmlRpc
{
  class MyXmlRpcValue : public XmlRpc::XmlRpcValue
  {
    public:
      operator std::map<std::string, XmlRpc::XmlRpcValue>&()     { assertTypeOrInvalid(TypeStruct); return *_value.asStruct; }
  };
}

#endif