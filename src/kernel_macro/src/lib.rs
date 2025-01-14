use proc_macro::TokenStream;
use quote::quote;
use syn::{ItemType, parse_macro_input};

#[proc_macro_attribute]
pub fn node_registration(attr: TokenStream, item: TokenStream) -> TokenStream {
    // 1) 解析属性参数
    let node_type = attr.to_string().replace("\"", "");

    // 2) 解析被标注的 type Item
    let item_clone = item.clone(); // 先克隆一份，因为 parse_macro_input 会消费 TokenStream
    let parsed_type = parse_macro_input!(item_clone as ItemType);
    let ident = &parsed_type.ident; // NewNode

    // 3) 生成新的代码 (保留原 type + 额外逻辑)
    quote! {
        // 原样输出原始的 type 声明
        #parsed_type

        ::inventory::submit!{
            NodeRegister{
                node_type: #node_type,
                node_creator: |robot_name: String, params: ::serde_json::Value| -> Box<dyn NodeExtBehavior>{
                    let name = format!("{}:{}", #node_type, robot_name);
                    Box::new(#ident::from_params(name, params))
                }
            }
        }
    }
}
