use proc_macro::TokenStream;
use quote::quote;
use syn::{GenericArgument, ItemType, PathArguments, Type, parse_macro_input};

#[proc_macro_attribute]
pub fn node_registration(attr: TokenStream, item: TokenStream) -> TokenStream {
    // 解析属性参数
    let node_type = attr.to_string().replace("\"", "");
    let input = parse_macro_input!(item as ItemType);
    let type_name = &input.ident;

    // 检查类型是否是 Node 的别名，并提取最后一个泛型参数
    let last_generic_param = match &*input.ty {
        Type::Path(type_path) => {
            // 获取路径中的最后一个段落
            let segment = type_path.path.segments.last().unwrap();
            // 提取泛型参数列表
            if let PathArguments::AngleBracketed(args) = &segment.arguments {
                // 获取最后一个泛型参数
                let last_arg = args.args.last().unwrap();
                // 提取泛型参数中的类型
                if let GenericArgument::Type(ty) = last_arg {
                    Some(ty)
                } else {
                    return syn::Error::new_spanned(last_arg, "Expected a type argument.")
                        .to_compile_error()
                        .into();
                }
            } else {
                return syn::Error::new_spanned(segment, "Expected angle-bracketed arguments.")
                    .to_compile_error()
                    .into();
            }
        }
        _ => {
            return syn::Error::new_spanned(
                &input.ty,
                "The #[node_registration] attribute can only be applied to type aliases.",
            )
            .to_compile_error()
            .into();
        }
    };

    // 生成代码
    let expanded = quote! {
        #input

        impl NodeExtBehavior<#last_generic_param> for #type_name{}

        inventory::submit! {
            NodeRegister::<#last_generic_param> {
                node_type: #node_type,
                node_creator: |name: String, params: serde_json::Value| -> Box<dyn NodeExtBehavior<#last_generic_param>> {
                    Box::new(#type_name::from_params(name, params))
                }
            }
        }
    };

    // 返回生成的代码
    TokenStream::from(expanded)
}
