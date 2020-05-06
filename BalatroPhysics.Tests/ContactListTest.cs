using BalatroPhysics.Dynamics;
using System;
using System.Collections.Generic;
using System.Linq;
using Xunit;

namespace BalatroPhysics.Tests
{
    public class ContactListTest
    {
        [Fact]
        public void ZeroListTest()
        {
            ContactList contacts = new ContactList();
            Assert.Empty(contacts);
        }

        [Fact]
        public void MaxTest()
        {
            ContactList contacts = new ContactList();

            for(int i = 0; i < ContactList.Max + 1; i++)
            {
                contacts.Add(new Contact());
            }
            Assert.Equal(ContactList.Max, contacts.Count);
        }

        [Fact]
        public void AddTest()
        {
            ContactList contacts = new ContactList();

            List<Contact> c = new List<Contact>();
            for(int i = 0; i < ContactList.Max + 1; i++)
            {
                Contact contact = new Contact();
                c.Add(contact);
                contacts.Add(contact);
            }
            
            for(int i = 0; i < c.Count - 1; i++)
            {
                Assert.Contains(c[i], contacts);
            }
            Assert.DoesNotContain(c[c.Count - 1], contacts);
        }

        [Fact]
        public void InsertTest()
        {
            Random random = new Random();
            ContactList contacts = new ContactList();

            List<Contact> c = new List<Contact>();
            // Inserts like a stack.
            for (int i = 0; i < ContactList.Max * 2; i++)
            {
                Contact contact = new Contact();
                c.Add(contact);
                contacts.Insert(0, contact);
            }

            // First half of the inserted elements should no longer be in the list.
            for (int i = 0; i < c.Count / 2; i++)
            {
                Assert.DoesNotContain(c[i], contacts);
            }

            // Second half of the inserted elements must be in the list.
            for (int i = c.Count / 2; i < c.Count; i++)
            {
                Assert.Contains(c[i], contacts);
            }

            // Insert new elements in the already filled list and check if they are there.
            for (int i = 0; i < ContactList.Max; i++)
            {
                Contact contact = new Contact();
                contacts.Insert(random.Next(0, ContactList.Max - 1), contact);
                Assert.Contains(contact, contacts);
            }
        }

        [Fact]
        public void ClearTest()
        {
            ContactList contacts = new ContactList();

            for (int i = 0; i < ContactList.Max; i++)
            {
                Contact contact = new Contact();
                contacts.Add(contact);
            }

            Assert.Equal(ContactList.Max, contacts.Count);

            contacts.Clear();

            Assert.Empty(contacts);
        }

        [Fact]
        public void ContainsTest()
        {
            ContactList contacts = new ContactList();

            for (int i = 0; i < ContactList.Max; i++)
            {
                Contact contact = new Contact();
                contacts.Add(contact);
                bool contains = contacts.Contains(contact);
                Assert.True(contains);
            }
        }

        [Fact]
        public void CopyToTest()
        {
            ContactList contacts = new ContactList();

            for (int i = 0; i < ContactList.Max; i++)
            {
                Contact contact = new Contact();
                contacts.Add(contact);
            }

            Contact[] copy = new Contact[contacts.Count];
            contacts.CopyTo(copy, 0);

            for(int i = 0; i < copy.Length; i++)
            {
                Assert.Equal(copy[i], contacts[i]);
            }
        }

        [Fact]
        public void IndexOfTest()
        {
            ContactList contacts = new ContactList();

            for (int i = 0; i < ContactList.Max; i++)
            {
                Contact contact = new Contact();
                contacts.Add(contact);
                Assert.Equal(i, contacts.IndexOf(contact));
            }
        }

        [Fact]
        public void RemoveTest()
        {
            ContactList contacts = new ContactList();
            List<Contact> c = new List<Contact>();
            for (int i = 0; i < ContactList.Max; i++)
            {
                Contact contact = new Contact();
                contacts.Add(contact);
                c.Add(contact);
            }

            int count = ContactList.Max;
            foreach(Contact contact in c)
            {
                Assert.Contains(contact, contacts);

                Assert.Equal(count, contacts.Count);
                contacts.Remove(contact);
                count -= 1;
                Assert.Equal(count, contacts.Count);

                Assert.DoesNotContain(contact, contacts);
            }
        }

        [Fact]
        public void RemoveAtTest()
        {
            ContactList contacts = new ContactList();
            List<Contact> c = new List<Contact>();
            for (int i = 0; i < ContactList.Max; i++)
            {
                Contact contact = new Contact();
                contacts.Add(contact);
                c.Add(contact);
            }

            for(int i = 0; i < ContactList.Max; i++)
            {
                Assert.Equal(c[i], contacts[0]);
                contacts.RemoveAt(0);
            }
        }
    }
}
