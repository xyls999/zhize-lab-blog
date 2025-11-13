import { defineClientConfig } from "vuepress/client";
import ECharts from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+plugin-markdown-c_26431bf46578b1eaadbdb6d15a4bfa4e/node_modules/@vuepress/plugin-markdown-chart/lib/client/components/ECharts.js";

export default defineClientConfig({
  enhance: ({ app }) => {
    app.component("ECharts", ECharts);
  },
});
