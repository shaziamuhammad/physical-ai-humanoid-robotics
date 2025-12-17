import React from 'react';
import Layout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      {/* Add the chatbot to all pages */}
      <Chatbot chapterId={typeof window !== 'undefined' ? window.location.pathname : ''} />
    </>
  );
}