import { CodeTabs } from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+plugin-markdown-t_7565f791ed263345c1b620485acd4dc1/node_modules/@vuepress/plugin-markdown-tab/lib/client/components/CodeTabs.js";
import { Tabs } from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+plugin-markdown-t_7565f791ed263345c1b620485acd4dc1/node_modules/@vuepress/plugin-markdown-tab/lib/client/components/Tabs.js";
import "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+plugin-markdown-t_7565f791ed263345c1b620485acd4dc1/node_modules/@vuepress/plugin-markdown-tab/lib/client/styles/vars.css";

export default {
  enhance: ({ app }) => {
    app.component("CodeTabs", CodeTabs);
    app.component("Tabs", Tabs);
  },
};
