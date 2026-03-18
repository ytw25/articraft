import * as React from "react";
import { cva, type VariantProps } from "class-variance-authority";

import { cn } from "@/lib/utils";

const buttonVariants = cva(
  "inline-flex items-center justify-center whitespace-nowrap rounded-sm text-[12px] font-medium transition-colors disabled:pointer-events-none disabled:opacity-50 focus-visible:outline-1 focus-visible:outline-offset-1 focus-visible:outline-[#007acc]",
  {
    variants: {
      variant: {
        default: "bg-[#1e1e1e] text-white hover:bg-[#333]",
        outline: "border border-[#e4e4e4] bg-white text-[#1e1e1e] hover:bg-[#f5f5f5]",
        ghost: "text-[#666] hover:bg-[#eee] hover:text-[#1e1e1e]",
      },
      size: {
        default: "h-8 px-3",
        sm: "h-7 px-2 text-[11px]",
      },
    },
    defaultVariants: {
      variant: "default",
      size: "default",
    },
  },
);

export type ButtonProps = React.ButtonHTMLAttributes<HTMLButtonElement> &
  VariantProps<typeof buttonVariants>;

export function Button({ className, variant, size, ...props }: ButtonProps): React.JSX.Element {
  return <button className={cn(buttonVariants({ variant, size, className }))} {...props} />;
}
